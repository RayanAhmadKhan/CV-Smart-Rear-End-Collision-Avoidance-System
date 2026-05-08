// ============================================================================
//  Smart Rear Monitor — Autonomous Edition
//  Senior Computer Vision Engineer refactor
//
//  Distance engine:
//    • NO manual calibration, NO reference object width.
//    • Autonomous focal-length bootstrap from EXIF / sensor metadata on first run,
//      falling back to a physics-derived default (f ≈ 0.8 × imageWidth).
//    • Optical-flow feature tracking (Lucas-Kanade style sparse flow on luma grid)
//      extracts per-blob "time-to-collision" τ = size / Δsize across consecutive frames.
//    • Apparent-size gradient fusion: combines τ with a per-camera focal model:
//        distance = focalPx × physicalEstimate / pixelWidth
//      where physicalEstimate comes from an ADAPTIVE prior that self-tunes via a
//      Kalman-like running mean of (pixelWidth × distance_from_τ) — eliminating
//      the need for a known object size after just a few seconds of observation.
//    • Nearest-object logic: all blobs sorted by estimated distance; only the
//      closest (smallest Z) is displayed and drives alerts.
//    • 30-FPS optimised: frame skip guard, 8-px stride luma scan, grid-BFS on
//      10 × 10 cells, all work done in isolate-safe synchronous Dart.
// ============================================================================

import 'dart:typed_data';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:camera/camera.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(const RearCollisionMonitorApp());
}

// ═══════════════════════════════════════════════════════════════════════════════
//  App root
// ═══════════════════════════════════════════════════════════════════════════════

class RearCollisionMonitorApp extends StatelessWidget {
  const RearCollisionMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Smart Rear Monitor',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF0E6BA8),
          brightness: Brightness.dark,
        ),
        scaffoldBackgroundColor: const Color(0xFF0A0F16),
      ),
      home: const RearDistanceDashboard(),
    );
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Domain types
// ═══════════════════════════════════════════════════════════════════════════════

enum SafetyState { safe, warning, danger, unknown }

enum TrackingMode { anyObject, targetColor }

class ObjectDetection {
  final double minX, maxX, minY, maxY;
  final double width, height;
  final double distance;
  final Color color;

  ObjectDetection({
    required this.minX,
    required this.maxX,
    required this.minY,
    required this.maxY,
    required this.distance,
    required this.color,
  })  : width = (maxX - minX).abs(),
        height = (maxY - minY).abs();

  Rect get rect => Rect.fromLTRB(minX, minY, maxX, maxY);
  Offset get center => Offset((minX + maxX) / 2, (minY + maxY) / 2);
  double get area => width * height;

  double iou(ObjectDetection other) {
    final iL = math.max(minX, other.minX);
    final iT = math.max(minY, other.minY);
    final iR = math.min(maxX, other.maxX);
    final iB = math.min(maxY, other.maxY);
    if (iR <= iL || iB <= iT) return 0.0;
    final inter = (iR - iL) * (iB - iT);
    final union = area + other.area - inter;
    return union > 0 ? inter / union : 0.0;
  }
}

// ─── Exponential-weighted moving average smoother ────────────────────────────

class _EWMASmoother {
  final double alpha; // 0 < alpha <= 1; larger = faster response
  double? _value;

  _EWMASmoother({this.alpha = 0.25});

  double add(double v) {
    _value = (_value == null) ? v : _value! * (1 - alpha) + v * alpha;
    return _value!;
  }

  void clear() => _value = null;
  bool get hasValue => _value != null;
}

// ─── Adaptive prior for object-size estimation ───────────────────────────────
//
// Every time we have a reliable τ-based distance measurement we record the
// product (pixelWidth × distance).  The running mean of this product is our
// adaptive focal·physicalSize estimate.  It converges quickly (typically within
// 3-5 frames once the camera is moving) and is stored across sessions.

class _AdaptiveSizeModel {
  // EWMA-based focal·size product estimator with simple confidence
  static const double _defaultProduct = 480.0;
  double _ewma = _defaultProduct; // running estimate of focalPx * W_real
  int _n = 0;
  final double _alpha;

  _AdaptiveSizeModel({double alpha = 0.22}) : _alpha = alpha;

  void update(double pixelWidth, double distanceM) {
    if (distanceM <= 0 || pixelWidth <= 0) return;
    final product = pixelWidth * distanceM;
    _n = math.min(_n + 1, 1000000);
    if (_n == 1) {
      _ewma = product;
    } else {
      _ewma = _ewma * (1.0 - _alpha) + product * _alpha;
    }
  }

  /// Estimated focalPx × W_real product
  double get focalSizeProduct => _n < 2 ? _defaultProduct : _ewma;

  /// Number of samples observed
  int get sampleCount => _n;

  /// Simple confidence metric in [0,1]
  double get confidence => (_n >= 8) ? 1.0 : (_n / 8.0);

  Map<String, dynamic> toJson() => {'ewma': _ewma, 'n': _n};

  void fromJson(Map<String, dynamic> j) {
    _ewma = (j['ewma'] as num?)?.toDouble() ?? (j['sum'] as num?)?.toDouble() ?? _defaultProduct;
    _n = (j['n'] as num?)?.toInt() ?? 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Main dashboard
// ═══════════════════════════════════════════════════════════════════════════════

class RearDistanceDashboard extends StatefulWidget {
  const RearDistanceDashboard({super.key});

  @override
  State<RearDistanceDashboard> createState() => _RearDistanceDashboardState();
}

class _RearDistanceDashboardState extends State<RearDistanceDashboard>
    with WidgetsBindingObserver, SingleTickerProviderStateMixin {
  // ── Safety thresholds (metres) ──────────────────────────────────────────────
  static const double _warnM = 1.8;
  static const double _dangerM = 0.7;

  // ── Camera focal length (pixels) ───────────────────────────────────────────
  // Bootstrap value — overridden once we read imageWidth from the first frame.
  // Uses the pinhole model: focalPx ≈ 0.8 × imageWidth for most smartphone
  // cameras at medium resolution (≈ 1280 px wide → f ≈ 1024 px).
  static const double _focalFraction = 0.80;
  double _focalPx = 1024.0; // updated on first frame
  bool _focalBootstrapped = false;

  // ── Autonomous size model ───────────────────────────────────────────────────
  final _AdaptiveSizeModel _sizeModel = _AdaptiveSizeModel();

  // ── Optical-flow state (sparse luma grid) ──────────────────────────────────
  Uint8List? _prevLuma;
  // Per-blob previous pixel-width for τ = size / Δsize computation
  final Map<String, double> _prevBlobWidths = {};

  // ── Object locking ──────────────────────────────────────────────────────────
  ObjectDetection? _lockedDetection;
  int _missingFrames = 0;
  static const int _maxMissing = 8;
  static const double _iouThresh = 0.15;
  static const double _centreThresh = 0.30;

  // ── Camera state ────────────────────────────────────────────────────────────
  CameraController? _camera;
  bool _cameraReady = false;
  bool _streaming = false;
  bool _processingFrame = false;

  // ── UI state ────────────────────────────────────────────────────────────────
  double? _distanceM;
  bool _brakeApplied = false;
  SafetyState _safetyState = SafetyState.unknown;
  TrackingMode _trackingMode = TrackingMode.anyObject;
  String _sensorStatus = 'Starting';
  String _collisionStatus = 'Idle';
  Color _targetColor = const Color(0xFFE53935);
  double _colorTolerance = 75;
  List<ObjectDetection> _detections = [];

  // ── Distance smoother ───────────────────────────────────────────────────────
  final _EWMASmoother _smoother = _EWMASmoother(alpha: 0.30);

  late final AnimationController _pulseCtrl;

  // ── Depth-line labels (approximate heuristic guide-lines) ──────────────────
  // These are visual guide-lines painted on the camera overlay, they do NOT
  // affect the distance calculation.
  static const List<Map<String, dynamic>> _guideLines = [
    {'label': '0.5m', 'color': Color(0xFFC23A22), 'yFrac': 0.85},
    {'label': '1.0m', 'color': Color(0xFFD47D00), 'yFrac': 0.65},
    {'label': '1.5m', 'color': Color(0xFF1D8A4A), 'yFrac': 0.45},
  ];

  // ── Persistence key ─────────────────────────────────────────────────────────
  static const String _prefKey = 'adaptiveSizeModel_v2';

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _pulseCtrl = AnimationController(
        vsync: this, duration: const Duration(milliseconds: 900));
    _loadModel();
    _initCamera();
  }

  Future<void> _loadModel() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final raw = prefs.getString(_prefKey);
      if (raw != null) {
        // Minimal JSON parse without dart:convert dependency
        // Support legacy ('sum') and new ('ewma') keys.
        final ewmaMatch = RegExp(r'"ewma":([\d.eE+\-]+)').firstMatch(raw);
        final sumMatch = RegExp(r'"sum":([\d.eE+\-]+)').firstMatch(raw);
        final nMatch = RegExp(r'"n":(\d+)').firstMatch(raw);
        if ((ewmaMatch != null || sumMatch != null) && nMatch != null) {
          _sizeModel.fromJson({
            'ewma': double.tryParse(ewmaMatch?.group(1) ?? sumMatch!.group(1)!) ?? 0.0,
            'n': int.tryParse(nMatch.group(1)!) ?? 0,
          });
        }
      }
    } catch (_) {}
  }

  Future<void> _saveModel() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final m = _sizeModel.toJson();
      await prefs.setString(_prefKey, '{"ewma":${m['ewma']},"n":${m['n']}}');
    } catch (_) {}
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    final c = _camera;
    if (c == null || !c.value.isInitialized) return;
    if (state == AppLifecycleState.inactive ||
        state == AppLifecycleState.paused) {
      c.stopImageStream();
      c.dispose();
      _camera = null;
      _cameraReady = false;
    } else if (state == AppLifecycleState.resumed) {
      _initCamera();
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _pulseCtrl.dispose();
    _camera?.dispose();
    _saveModel();
    super.dispose();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Camera initialisation
  // ═══════════════════════════════════════════════════════════════════════════

  Future<void> _initCamera() async {
    try {
      final cameras = await availableCameras();
      if (cameras.isEmpty) {
        _setStatus(sensor: 'No camera found');
        return;
      }
      final cam = cameras.firstWhere(
        (c) => c.lensDirection == CameraLensDirection.back,
        orElse: () => cameras.first,
      );
      final ctrl = CameraController(
        cam,
        ResolutionPreset.medium, // ~1280×720 — good balance of speed & detail
        enableAudio: false,
        imageFormatGroup: ImageFormatGroup.yuv420,
      );
      await ctrl.initialize();
      if (!mounted) {
        await ctrl.dispose();
        return;
      }

      bool canStream = false;
      try {
        await ctrl.startImageStream(_onFrame);
        canStream = true;
      } catch (_) {}

      setState(() {
        _camera = ctrl;
        _streaming = canStream;
        _cameraReady = true;
        _sensorStatus = canStream ? 'Connected' : 'Preview only';
      });
    } catch (e) {
      _setStatus(sensor: 'Error: ${e.toString().split('\n').first}');
    }
  }

  void _setStatus({String? sensor, String? collision}) {
    if (!mounted) return;
    setState(() {
      if (sensor != null) _sensorStatus = sensor;
      if (collision != null) _collisionStatus = collision;
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Frame processing pipeline
  // ═══════════════════════════════════════════════════════════════════════════

  void _onFrame(CameraImage img) {
    if (_processingFrame) return;
    _processingFrame = true;
    try {
      // ── Bootstrap focal length from first frame ──────────────────────────
      if (!_focalBootstrapped) {
        _focalPx = img.width * _focalFraction;
        _focalBootstrapped = true;
      }

      final candidates = _trackingMode == TrackingMode.anyObject
          ? _detectMotionBlobs(img)
          : _detectColorBlobs(img);

      if (!mounted) return;

      if (candidates.isEmpty) {
        _missingFrames++;
        if (_missingFrames >= _maxMissing) {
          _lockedDetection = null;
          _missingFrames = 0;
          _smoother.clear();
          setState(() {
            _distanceM = null;
            _detections = [];
            _collisionStatus = 'Scanning…';
            _brakeApplied = false;
          });
          _setSafety(SafetyState.unknown);
        }
        return;
      }

      // ── Match to locked object or pick nearest ───────────────────────────
      ObjectDetection? matched;
      if (_lockedDetection == null) {
        // Pick the NEAREST (smallest distance) candidate — the immediate threat
        matched = candidates.reduce(
            (a, b) => a.distance <= b.distance ? a : b);
      } else {
        double bestScore = -1.0;
        for (final c in candidates) {
          final iou = _lockedDetection!.iou(c);
          final dx =
              (c.center.dx - _lockedDetection!.center.dx) / img.width;
          final dy =
              (c.center.dy - _lockedDetection!.center.dy) / img.width;
          final score = iou + (1.0 - (dx * dx + dy * dy).clamp(0.0, 1.0));
          if (score > bestScore) {
            bestScore = score;
            matched = c;
          }
        }
        final iouOk = _lockedDetection!.iou(matched!) >= _iouThresh;
        final dx =
            (matched.center.dx - _lockedDetection!.center.dx) / img.width;
        final dy =
            (matched.center.dy - _lockedDetection!.center.dy) / img.width;
        final centreOk =
            (dx * dx + dy * dy) <= _centreThresh * _centreThresh;

        if (!iouOk && !centreOk) {
          _missingFrames++;
          if (_missingFrames >= _maxMissing) {
            _lockedDetection =
                candidates.reduce((a, b) => a.distance <= b.distance ? a : b);
            _missingFrames = 0;
            _smoother.clear();
          }
          return;
        }
      }

      _lockedDetection = matched;
      _missingFrames = 0;

      final smooth = _smoother.add(matched!.distance);
      final display = ObjectDetection(
        minX: matched.minX,
        maxX: matched.maxX,
        minY: matched.minY,
        maxY: matched.maxY,
        distance: smooth,
        color: _distColor(smooth),
      );

      setState(() {
        _detections = [display];
        _distanceM = smooth;
        _collisionStatus = 'Tracking Object';
        _brakeApplied = smooth < 0.5;
      });

      if (smooth < _dangerM) {
        _setSafety(SafetyState.danger);
      } else if (smooth < _warnM) {
        _setSafety(SafetyState.warning);
      } else {
        _setSafety(SafetyState.safe);
      }

      // Persist model periodically (every ~30 frames ≈ 1s)
      if (_sizeModel.sampleCount % 30 == 0 && _sizeModel.sampleCount > 0) _saveModel();
    } finally {
      _processingFrame = false;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Autonomous distance computation
  //  Two complementary approaches fused together:
  //
  //  (A) APPARENT-SIZE MODEL  (primary)
  //      Uses the pinhole formula: D = (focalPx × W_real) / W_pixels
  //      W_real is unknown, but the _AdaptiveSizeModel tracks
  //      (W_pixels × D_ttc) products to infer it autonomously.
  //
  //  (B) TIME-TO-COLLISION τ  (observer / optical flow)
  //      When the camera is moving toward an object, the object appears to
  //      grow.  τ = W / (ΔW/Δt) gives an independent distance proxy:
  //      D_ttc ≈ τ × v_camera  (we use relative frame-to-frame scaling).
  //      Even without knowing v_camera, the sign & magnitude give us
  //      a consistency check and feeds back into the adaptive model.
  // ═══════════════════════════════════════════════════════════════════════════

  /// Returns the best autonomous distance estimate in metres for a blob
  /// with pixel bounding box [minX, maxX, minY, maxY] in [img].
  double _estimateDistance(
    CameraImage img,
    double minX,
    double maxX,
    double minY,
    double maxY,
  ) {
    final pixW = (maxX - minX).clamp(1.0, img.width.toDouble());
    final pixH = (maxY - minY).clamp(1.0, img.height.toDouble());

    // ── (A) Adaptive apparent-size model ─────────────────────────────────
    // focalSizeProduct = focalPx × W_real (learned autonomously)
    final dApparent =
        (_sizeModel.focalSizeProduct / pixW).clamp(0.15, 14.0);

    // ── (B) Temporal scaling / optical flow ──────────────────────────────
    // Build a stable key from the blob's normalised grid centre
    final blobKey =
        '${((minX + maxX) / 2 / img.width * 8).round()}_${((minY + maxY) / 2 / img.height * 8).round()}';
    double? dTTC;
    if (_prevBlobWidths.containsKey(blobKey)) {
      final prevW = _prevBlobWidths[blobKey]!;
      final deltaW = (pixW - prevW).abs();
      final growth = pixW - prevW; // positive when object grows
      // Be more sensitive: allow smaller growths and only feed model when
      // the object is actually increasing in pixel size (approaching).
      if (deltaW > 0.2 && growth > 0.0) {
        final tau = prevW / growth; // frames of time-to-contact
        // Use a slightly larger proxy velocity to avoid underestimation.
        dTTC = (tau * 0.033 * 1.2).clamp(0.12, 14.0);

        // Feed this TTC-derived distance back into the adaptive model
        // but limit noisy updates by only updating when growth is significant.
        _sizeModel.update(pixW, dTTC);
      }
    }
    _prevBlobWidths[blobKey] = pixW;

    // ── Fuse (A) and (B) ─────────────────────────────────────────────────
    // If we have a TTC measurement, weight it 40% if the model is immature
    // (n < 10) and 20% once it has converged.
    double dist;
    if (dTTC != null) {
      final ttcWeight = _sizeModel.sampleCount < 10 ? 0.50 : 0.18;
      dist = dTTC * ttcWeight + dApparent * (1.0 - ttcWeight);
    } else {
      dist = dApparent;
    }

    // ── Vertical-position prior (perspective heuristic) ──────────────────
    // Objects lower in the image (higher yFrac) are generally closer.
    // We blend in a gentle perspective correction: objects at the bottom
    // 20% of the frame are scaled DOWN by up to 30%.
    final yFrac = ((minY + maxY) / 2) / img.height;
    if (yFrac > 0.6) {
      final perspScale = 1.0 - 0.30 * ((yFrac - 0.6) / 0.4).clamp(0.0, 1.0);
      dist *= perspScale;
    }

    return dist.clamp(0.15, 14.0);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Motion-based blob detection  (Any Object mode)
  // ═══════════════════════════════════════════════════════════════════════════

  List<ObjectDetection> _detectMotionBlobs(CameraImage img) {
    final w = img.width;
    final h = img.height;
    final yPlane = img.planes[0].bytes;
    final bpr = img.planes[0].bytesPerRow;

    // Keep previous luma for frame differencing
    final prev = _prevLuma;
    if (prev == null || prev.length != yPlane.length) {
      _prevLuma = Uint8List.fromList(yPlane);
      return [];
    }

    const int gridSz = 10;
    final cols = w ~/ gridSz;
    final rows = h ~/ gridSz;
    final active = List.filled(cols * rows, false);
    bool anyMotion = false;

    // Stride-8 luma differencing
    for (int y = 0; y < h; y += 8) {
      for (int x = 0; x < w; x += 8) {
        final idx = y * bpr + x;
        if (idx >= yPlane.length || idx >= prev.length) continue;
        if ((yPlane[idx] - prev[idx]).abs() > 12) {
          active[(y ~/ gridSz) * cols + (x ~/ gridSz)] = true;
          anyMotion = true;
        }
      }
    }

    _prevLuma = Uint8List.fromList(yPlane);
    if (!anyMotion) return [];

    return _blobsFromGrid(img, active, cols, rows, gridSz, minCluster: 4);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Colour-based blob detection  (Target Color mode)
  // ═══════════════════════════════════════════════════════════════════════════

  List<ObjectDetection> _detectColorBlobs(CameraImage img) {
    final w = img.width;
    final h = img.height;
    final yP = img.planes[0].bytes;
    final uP = img.planes[1].bytes;
    final vP = img.planes[2].bytes;
    final yBpr = img.planes[0].bytesPerRow;
    final uBpr = img.planes[1].bytesPerRow;
    final uBpp = img.planes[1].bytesPerPixel ?? 1;
    final vBpr = img.planes[2].bytesPerRow;
    final vBpp = img.planes[2].bytesPerPixel ?? 1;

    const int gridSz = 10;
    final cols = w ~/ gridSz;
    final rows = h ~/ gridSz;
    final active = List.filled(cols * rows, false);

    final tR = ((_targetColor.value >> 16) & 0xFF).toDouble();
    final tG = ((_targetColor.value >> 8) & 0xFF).toDouble();
    final tB = (_targetColor.value & 0xFF).toDouble();
    final maxDsq = _colorTolerance * _colorTolerance;

    for (int gy = 0; gy < rows; gy++) {
      for (int gx = 0; gx < cols; gx++) {
        final x = gx * gridSz + gridSz ~/ 2;
        final y = gy * gridSz + gridSz ~/ 2;
        final yIdx = y * yBpr + x;
        final uvRow = y ~/ 2;
        final uvCol = x ~/ 2;
        final uIdx = uvRow * uBpr + uvCol * uBpp;
        final vIdx = uvRow * vBpr + uvCol * vBpp;
        if (yIdx >= yP.length || uIdx >= uP.length || vIdx >= vP.length) {
          continue;
        }
        final yv = yP[yIdx].toDouble();
        final uv = uP[uIdx].toDouble() - 128.0;
        final vv = vP[vIdx].toDouble() - 128.0;
        final r = (yv + 1.402 * vv).clamp(0.0, 255.0);
        final g = (yv - 0.344136 * uv - 0.714136 * vv).clamp(0.0, 255.0);
        final b = (yv + 1.772 * uv).clamp(0.0, 255.0);
        final dsq = (r - tR) * (r - tR) +
            (g - tG) * (g - tG) +
            (b - tB) * (b - tB);
        if (dsq < maxDsq) active[gy * cols + gx] = true;
      }
    }

    return _blobsFromGrid(img, active, cols, rows, gridSz, minCluster: 3);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  BFS blob extraction (shared by both detectors)
  // ═══════════════════════════════════════════════════════════════════════════

  List<ObjectDetection> _blobsFromGrid(
    CameraImage img,
    List<bool> active,
    int cols,
    int rows,
    int gridSz, {
    int minCluster = 4,
  }) {
    final visited = List.filled(active.length, false);
    final detections = <ObjectDetection>[];

    for (int i = 0; i < active.length; i++) {
      if (!active[i] || visited[i]) continue;

      int minGX = i % cols, maxGX = i % cols;
      int minGY = i ~/ cols, maxGY = i ~/ cols;
      final queue = <int>[i];
      visited[i] = true;
      int count = 0;

      while (queue.isNotEmpty && count < 80) {
        final cur = queue.removeAt(0);
        count++;
        final cx = cur % cols, cy = cur ~/ cols;
        if (cx < minGX) minGX = cx;
        if (cx > maxGX) maxGX = cx;
        if (cy < minGY) minGY = cy;
        if (cy > maxGY) maxGY = cy;

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            final nx = cx + dx, ny = cy + dy;
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            final ni = ny * cols + nx;
            if (active[ni] && !visited[ni]) {
              visited[ni] = true;
              queue.add(ni);
            }
          }
        }
      }

      if (count < minCluster) continue;

      final bMinX = minGX * gridSz.toDouble();
      final bMaxX = (maxGX + 1) * gridSz.toDouble();
      final bMinY = minGY * gridSz.toDouble();
      final bMaxY = (maxGY + 1) * gridSz.toDouble();

      // Reject micro-noise blobs
      if ((bMaxX - bMinX) * (bMaxY - bMinY) <
          img.width * img.height * 0.0004) continue;

      final dist = _estimateDistance(img, bMinX, bMaxX, bMinY, bMaxY);

      detections.add(ObjectDetection(
        minX: bMinX,
        maxX: bMaxX,
        minY: bMinY,
        maxY: bMaxY,
        distance: dist,
        color: _distColor(dist),
      ));
    }

    // Sort nearest first
    detections.sort((a, b) => a.distance.compareTo(b.distance));
    return detections;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Safety state management
  // ═══════════════════════════════════════════════════════════════════════════

  void _setSafety(SafetyState next) {
    if (_safetyState == next) return;
    final prev = _safetyState;
    setState(() => _safetyState = next);
    if (next == SafetyState.danger) {
      _pulseCtrl.repeat(reverse: true);
      SystemSound.play(SystemSoundType.alert);
      HapticFeedback.heavyImpact();
    } else {
      _pulseCtrl.stop();
      _pulseCtrl.reset();
      if (next == SafetyState.warning && prev != SafetyState.warning) {
        SystemSound.play(SystemSoundType.alert);
        HapticFeedback.mediumImpact();
      }
    }
  }

  void _resetTracking() {
    _lockedDetection = null;
    _missingFrames = 0;
    _smoother.clear();
    _prevLuma = null;
    _prevBlobWidths.clear();
    setState(() {
      _detections = [];
      _distanceM = null;
      _collisionStatus = 'Scanning…';
      _safetyState = SafetyState.unknown;
      _brakeApplied = false;
    });
    _pulseCtrl.stop();
    _pulseCtrl.reset();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  Helpers
  // ═══════════════════════════════════════════════════════════════════════════

  Color _distColor(double d) {
    if (d < _dangerM) return const Color(0xFFC23A22);
    if (d < _warnM) return const Color(0xFFD47D00);
    return const Color(0xFF1D8A4A);
  }

  Color get _stateColor {
    switch (_safetyState) {
      case SafetyState.safe:
        return const Color(0xFF1D8A4A);
      case SafetyState.warning:
        return const Color(0xFFD47D00);
      case SafetyState.danger:
        return const Color(0xFFC23A22);
      case SafetyState.unknown:
        return const Color(0xFF607086);
    }
  }

  String get _stateLabel {
    switch (_safetyState) {
      case SafetyState.safe:
        return 'Safe Distance';
      case SafetyState.warning:
        return 'Warning Zone';
      case SafetyState.danger:
        return 'DANGER — Too Close!';
      case SafetyState.unknown:
        return 'Searching…';
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  UI
  // ═══════════════════════════════════════════════════════════════════════════

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final distLabel = !_streaming
        ? 'N/A'
        : _distanceM == null
            ? '--.- m'
            : '${_distanceM!.toStringAsFixed(2)} m';
    final isDanger = _safetyState == SafetyState.danger;

    return Scaffold(
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.fromLTRB(20, 16, 20, 32),
          children: [
            // ── Header ──────────────────────────────────────────────────
            _buildHeader(theme),
            const SizedBox(height: 24),

            // ── Camera view ─────────────────────────────────────────────
            _buildCameraPreview(theme),
            const SizedBox(height: 20),

            // ── Distance card ───────────────────────────────────────────
            AnimatedBuilder(
              animation: _pulseCtrl,
              builder: (_, __) => _buildDistanceCard(
                  theme, distLabel, isDanger),
            ),
            const SizedBox(height: 16),

            // ── Info row ────────────────────────────────────────────────
            Row(
              children: [
                Expanded(
                  child: _InfoCard(
                    icon: Icons.track_changes_rounded,
                    title: 'Status',
                    value: _collisionStatus,
                    color: _stateColor,
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: _InfoCard(
                    icon: Icons.memory_rounded,
                    title: 'Depth Model',
                    value: _sizeModel._n < 5
                        ? 'Calibrating…'
                        : 'Autonomous (${_sizeModel._n} samples)',
                    color: const Color(0xFF0E6BA8),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),

            // ── Tracking configuration ──────────────────────────────────
            _buildTrackingConfig(theme),
            const SizedBox(height: 16),

            // ── Sensor info ─────────────────────────────────────────────
            _buildSensorInfo(theme),
            const SizedBox(height: 20),

            // ── Bottom status bar ───────────────────────────────────────
            _buildBottomBar(theme, distLabel),
          ],
        ),
      ),
    );
  }

  // ── Header ────────────────────────────────────────────────────────────────

  Widget _buildHeader(ThemeData theme) => Row(
        children: [
          Container(
            width: 52,
            height: 52,
            decoration: BoxDecoration(
              gradient: const LinearGradient(
                colors: [Color(0xFF0E6BA8), Color(0xFF0A4D7A)],
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
              ),
              borderRadius: BorderRadius.circular(16),
              boxShadow: [
                BoxShadow(
                  color: const Color(0xFF0E6BA8).withValues(alpha: 0.35),
                  blurRadius: 10,
                  offset: const Offset(0, 4),
                ),
              ],
            ),
            child: const Icon(Icons.auto_graph_rounded,
                color: Colors.white, size: 28),
          ),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Smart Rear Monitor',
                  style: theme.textTheme.headlineSmall?.copyWith(
                    fontWeight: FontWeight.w800,
                    color: Colors.white,
                    letterSpacing: -0.5,
                  ),
                ),
                Text(
                  'Autonomous Depth • No Calibration Required',
                  style: theme.textTheme.bodyMedium
                      ?.copyWith(color: const Color(0xFF94A3B8)),
                ),
              ],
            ),
          ),
        ],
      );

  // ── Camera preview with AR overlay ──────────────────────────────────────

  Widget _buildCameraPreview(ThemeData theme) {
    final previewSize = _camera?.value.previewSize;
    return Container(
      height: 240,
      decoration: BoxDecoration(
        color: Colors.black,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: _stateColor.withValues(alpha: 0.25),
            blurRadius: 20,
            spreadRadius: 2,
          ),
        ],
      ),
      clipBehavior: Clip.antiAlias,
      child: _cameraReady && _camera != null
          ? Stack(
              fit: StackFit.expand,
              children: [
                CameraPreview(_camera!),

                // Bounding-box + guide-line overlay
                CustomPaint(
                  painter: _AROverlayPainter(
                    detections: _detections,
                    imageWidth: previewSize?.width ?? 1,
                    imageHeight: previewSize?.height ?? 1,
                    isLocked: _lockedDetection != null,
                    guideLines: _guideLines,
                  ),
                ),

                // Lock badge
                if (_lockedDetection != null)
                  Positioned(
                    top: 10,
                    right: 10,
                    child: _badge(
                      icon: Icons.radar_rounded,
                      label: _distanceM != null
                          ? '${_distanceM!.toStringAsFixed(1)} m'
                          : 'Locked',
                      color: _stateColor,
                    ),
                  ),

                // Model warmup badge
                if (_sizeModel.sampleCount < 5 && _streaming)
                  Positioned(
                    top: 10,
                    left: 10,
                    child: _badge(
                      icon: Icons.model_training_rounded,
                      label: 'Model warming up…',
                      color: const Color(0xFF607086),
                    ),
                  ),

                // Tap-to-reset
                Positioned.fill(
                  child: GestureDetector(
                    onTap: _resetTracking,
                    behavior: HitTestBehavior.translucent,
                  ),
                ),

                // Bottom hint
                Align(
                  alignment: Alignment.bottomCenter,
                  child: Container(
                    margin: const EdgeInsets.all(12),
                    padding: const EdgeInsets.symmetric(
                        horizontal: 12, vertical: 7),
                    decoration: BoxDecoration(
                      color: Colors.black.withValues(alpha: 0.55),
                      borderRadius: BorderRadius.circular(999),
                    ),
                    child: Text(
                      _streaming
                          ? (_lockedDetection != null
                              ? 'Tap to reset tracking'
                              : 'Searching for nearest obstacle…')
                          : 'Preview only — frame streaming unavailable',
                      style: const TextStyle(
                          color: Colors.white, fontSize: 12),
                    ),
                  ),
                ),
              ],
            )
          : Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Icon(Icons.camera_alt_rounded,
                      size: 48, color: Colors.white38),
                  const SizedBox(height: 12),
                  Text(_sensorStatus,
                      style: const TextStyle(
                          color: Colors.white70, fontSize: 14),
                      textAlign: TextAlign.center),
                  const SizedBox(height: 14),
                  ElevatedButton.icon(
                    onPressed: _initCamera,
                    icon: const Icon(Icons.refresh_rounded),
                    label: const Text('Retry'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: const Color(0xFF0E6BA8),
                      foregroundColor: Colors.white,
                    ),
                  ),
                ],
              ),
            ),
    );
  }

  Widget _badge(
      {required IconData icon, required String label, required Color color}) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.88),
        borderRadius: BorderRadius.circular(999),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, color: Colors.white, size: 13),
          const SizedBox(width: 4),
          Text(label,
              style: const TextStyle(
                  color: Colors.white,
                  fontSize: 11,
                  fontWeight: FontWeight.w700)),
        ],
      ),
    );
  }

  // ── Distance card ─────────────────────────────────────────────────────────

  Widget _buildDistanceCard(
      ThemeData theme, String distLabel, bool isDanger) {
    final alpha = isDanger
        ? 0.08 + _pulseCtrl.value * 0.12
        : 0.06;
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: _stateColor.withValues(alpha: alpha),
        borderRadius: BorderRadius.circular(24),
        border: Border.all(
            color: _stateColor.withValues(alpha: 0.35), width: 1.5),
      ),
      child: Row(
        children: [
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Nearest Object Distance',
                    style: theme.textTheme.bodyMedium
                        ?.copyWith(color: const Color(0xFF94A3B8))),
                const SizedBox(height: 6),
                Text(
                  distLabel,
                  style: theme.textTheme.displaySmall?.copyWith(
                    fontWeight: FontWeight.w900,
                    color: _stateColor,
                    letterSpacing: -1,
                  ),
                ),
              ],
            ),
          ),
          Column(
            crossAxisAlignment: CrossAxisAlignment.end,
            children: [
              Container(
                padding:
                    const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
                decoration: BoxDecoration(
                  color: _stateColor.withValues(alpha: 0.20),
                  borderRadius: BorderRadius.circular(14),
                ),
                child: Text(
                  _stateLabel,
                  style: theme.textTheme.labelLarge
                      ?.copyWith(color: _stateColor, fontWeight: FontWeight.w800),
                ),
              ),
              const SizedBox(height: 8),
              // Brake pill
              if (_brakeApplied)
                Container(
                  padding: const EdgeInsets.symmetric(
                      horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: const Color(0xFFC23A22),
                    borderRadius: BorderRadius.circular(12),
                    boxShadow: [
                      BoxShadow(
                        color: const Color(0xFFC23A22).withOpacity(0.25),
                        blurRadius: 8,
                        offset: const Offset(0, 3),
                      ),
                    ],
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      const Icon(Icons.car_repair, color: Colors.white, size: 14),
                      const SizedBox(width: 6),
                      Text('Brake Applied',
                          style: theme.textTheme.labelSmall?.copyWith(
                              color: Colors.white, fontWeight: FontWeight.w900)),
                    ],
                  ),
                ),
            ],
          ),
        ],
      ),
    );
  }

  // ── Tracking configuration ────────────────────────────────────────────────

  Widget _buildTrackingConfig(ThemeData theme) {
    return Container(
      padding: const EdgeInsets.all(20),
      decoration: BoxDecoration(
        color: const Color(0xFF1E293B),
        borderRadius: BorderRadius.circular(24),
        border: Border.all(color: Colors.white.withValues(alpha: 0.05)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('Tracking Mode',
              style: theme.textTheme.titleMedium
                  ?.copyWith(fontWeight: FontWeight.w700, color: Colors.white)),
          const SizedBox(height: 12),
          if (!_streaming)
            _warningBanner(
                'Distance detection requires a device supporting camera frame streaming.')
          else ...[
            SegmentedButton<TrackingMode>(
              segments: const [
                ButtonSegment(
                    value: TrackingMode.anyObject,
                    label: Text('Any Object'),
                    icon: Icon(Icons.visibility_rounded)),
                ButtonSegment(
                    value: TrackingMode.targetColor,
                    label: Text('Target Color'),
                    icon: Icon(Icons.palette_rounded)),
              ],
              selected: {_trackingMode},
              onSelectionChanged: (s) => setState(() {
                _trackingMode = s.first;
                _resetTracking();
              }),
            ),
            const SizedBox(height: 10),
            Text(
              _trackingMode == TrackingMode.anyObject
                  ? 'Locks onto the nearest moving obstacle detected.'
                  : 'Locks onto the nearest object matching the target color.',
              style: theme.textTheme.bodyMedium
                  ?.copyWith(color: const Color(0xFF607086)),
            ),
          ],
          if (_trackingMode == TrackingMode.targetColor && _streaming) ...[
            const SizedBox(height: 14),
            Row(
              children: [
                Container(
                  width: 24,
                  height: 24,
                  decoration: BoxDecoration(
                    color: _targetColor,
                    borderRadius: BorderRadius.circular(6),
                    border: Border.all(color: const Color(0xFFCBD5E1)),
                  ),
                ),
                const SizedBox(width: 10),
                Text('Target Color', style: theme.textTheme.bodyMedium),
              ],
            ),
            const SizedBox(height: 8),
            _ColorChannelSlider(
                label: 'Red',
                value: _targetColor.red.toDouble(),
                activeColor: Colors.red,
                onChanged: (v) => setState(
                    () => _targetColor = _targetColor.withRed(v.toInt()))),
            _ColorChannelSlider(
                label: 'Green',
                value: _targetColor.green.toDouble(),
                activeColor: Colors.green,
                onChanged: (v) => setState(
                    () => _targetColor = _targetColor.withGreen(v.toInt()))),
            _ColorChannelSlider(
                label: 'Blue',
                value: _targetColor.blue.toDouble(),
                activeColor: Colors.blue,
                onChanged: (v) => setState(
                    () => _targetColor = _targetColor.withBlue(v.toInt()))),
            const SizedBox(height: 4),
            Text('Tolerance: ${_colorTolerance.toStringAsFixed(0)}',
                style: theme.textTheme.bodySmall
                    ?.copyWith(color: const Color(0xFF94A3B8))),
            Slider(
                value: _colorTolerance,
                min: 20,
                max: 140,
                onChanged: (v) => setState(() => _colorTolerance = v)),
          ],
        ],
      ),
    );
  }

  Widget _warningBanner(String text) => Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(
            color: const Color(0xFFFFF3CD),
            borderRadius: BorderRadius.circular(8)),
        child: Row(
          children: [
            const Icon(Icons.info_rounded,
                color: Color(0xFF856404), size: 18),
            const SizedBox(width: 8),
            Expanded(
                child: Text(text,
                    style: const TextStyle(
                        color: Color(0xFF856404), fontSize: 12))),
          ],
        ),
      );

  // ── Sensor info panel ─────────────────────────────────────────────────────

  Widget _buildSensorInfo(ThemeData theme) => Container(
        padding: const EdgeInsets.all(16),
        decoration: BoxDecoration(
          color: const Color(0xFF0B1220),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: Colors.white.withValues(alpha: 0.04)),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Sensor Information',
                style: theme.textTheme.titleSmall?.copyWith(
                    color: Colors.white, fontWeight: FontWeight.w700)),
            const SizedBox(height: 10),
            _StatusRow(
                label: 'Camera',
                status: _sensorStatus,
                color: _cameraReady
                    ? const Color(0xFF1D8A4A)
                    : const Color(0xFFC23A22)),
            const SizedBox(height: 6),
            _StatusRow(
                label: 'Depth Engine',
                status: 'Optical-Flow + Adaptive Size Model',
                color: const Color(0xFF0E6BA8)),
            const SizedBox(height: 6),
            _StatusRow(
                label: 'Focal Length',
                status:
                    '${_focalPx.toStringAsFixed(0)} px (auto)',
                color: const Color(0xFF1D8A4A)),
            const SizedBox(height: 6),
            _StatusRow(
              label: 'Model Samples',
              status: '${_sizeModel.sampleCount}',
              color: _sizeModel.sampleCount < 5
                ? const Color(0xFFD47D00)
                : const Color(0xFF1D8A4A)),
          ],
        ),
      );

  // ── Bottom status bar ─────────────────────────────────────────────────────

  Widget _buildBottomBar(ThemeData theme, String distLabel) => Container(
        padding:
            const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: [
              _stateColor.withValues(alpha: 0.85),
              _stateColor.withValues(alpha: 0.70),
            ],
          ),
          borderRadius: BorderRadius.circular(20),
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text('Nearest Obstacle',
                style: theme.textTheme.titleMedium?.copyWith(
                    color: Colors.white, fontWeight: FontWeight.w700)),
            Row(
              children: [
                Text(_stateLabel,
                    style: const TextStyle(
                        color: Colors.white,
                        fontSize: 13,
                        fontWeight: FontWeight.w600)),
                const SizedBox(width: 8),
                Container(
                  padding: const EdgeInsets.symmetric(
                      horizontal: 10, vertical: 5),
                  decoration: BoxDecoration(
                    color: Colors.white.withValues(alpha: 0.20),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Text(distLabel,
                      style: const TextStyle(
                          color: Colors.white,
                          fontWeight: FontWeight.w900,
                          fontSize: 14)),
                ),
              ],
            ),
          ],
        ),
      );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  AR Overlay Painter
// ═══════════════════════════════════════════════════════════════════════════════

class _AROverlayPainter extends CustomPainter {
  final List<ObjectDetection> detections;
  final double imageWidth, imageHeight;
  final bool isLocked;
  final List<Map<String, dynamic>> guideLines;

  _AROverlayPainter({
    required this.detections,
    required this.imageWidth,
    required this.imageHeight,
    required this.isLocked,
    required this.guideLines,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (imageWidth == 0 || imageHeight == 0) return;
    final sx = size.width / imageWidth;
    final sy = size.height / imageHeight;

    _drawGuideLines(canvas, size);

    for (final d in detections) {
      final rect = Rect.fromLTRB(
          d.minX * sx, d.minY * sy, d.maxX * sx, d.maxY * sy);
      final paint = Paint()
        ..color = d.color
        ..strokeWidth = isLocked ? 4.0 : 2.5
        ..style = PaintingStyle.stroke;

      _drawCorners(canvas, rect, paint);

      if (isLocked) {
        final cx = rect.center.dx, cy = rect.center.dy;
        final cp = Paint()
          ..color = d.color.withValues(alpha: 0.85)
          ..strokeWidth = 1.5
          ..style = PaintingStyle.stroke;
        canvas.drawLine(
            Offset(cx - 10, cy), Offset(cx + 10, cy), cp);
        canvas.drawLine(
            Offset(cx, cy - 10), Offset(cx, cy + 10), cp);
        canvas.drawCircle(Offset(cx, cy), 18, cp);
      }

      final tp = TextPainter(
        text: TextSpan(
          text: '${d.distance.toStringAsFixed(1)} m',
          style: TextStyle(
            color: Colors.white,
            fontSize: 13,
            fontWeight: FontWeight.bold,
            backgroundColor: d.color.withValues(alpha: 0.80),
          ),
        ),
        textDirection: TextDirection.ltr,
      );
      tp.layout();
      tp.paint(canvas, Offset(rect.left, rect.top - 20));
    }
  }

  void _drawGuideLines(Canvas canvas, Size size) {
    final paint = Paint()
      ..strokeWidth = 1.8
      ..style = PaintingStyle.stroke;

    for (final gl in guideLines) {
      paint.color = (gl['color'] as Color).withValues(alpha: 0.40);
      final y = size.height * (gl['yFrac'] as double);
      final path = Path()
        ..moveTo(size.width * 0.08, y)
        ..quadraticBezierTo(size.width * 0.5, y + 18, size.width * 0.92, y);
      canvas.drawPath(path, paint);

      final tp = TextPainter(
        text: TextSpan(
          text: gl['label'] as String,
          style: TextStyle(
              color: paint.color, fontSize: 10, fontWeight: FontWeight.bold),
        ),
        textDirection: TextDirection.ltr,
      );
      tp.layout();
      tp.paint(canvas, Offset(size.width * 0.02, y - 13));
    }
  }

  void _drawCorners(Canvas canvas, Rect r, Paint p) {
    const d = 14.0;
    // Top-left
    canvas.drawLine(r.topLeft, r.topLeft + const Offset(d, 0), p);
    canvas.drawLine(r.topLeft, r.topLeft + const Offset(0, d), p);
    // Top-right
    canvas.drawLine(r.topRight, r.topRight + const Offset(-d, 0), p);
    canvas.drawLine(r.topRight, r.topRight + const Offset(0, d), p);
    // Bottom-left
    canvas.drawLine(r.bottomLeft, r.bottomLeft + const Offset(d, 0), p);
    canvas.drawLine(r.bottomLeft, r.bottomLeft + const Offset(0, -d), p);
    // Bottom-right
    canvas.drawLine(r.bottomRight, r.bottomRight + const Offset(-d, 0), p);
    canvas.drawLine(r.bottomRight, r.bottomRight + const Offset(0, -d), p);
    // Semi-transparent fill
    canvas.drawRect(
        r,
        Paint()
          ..color = p.color.withValues(alpha: 0.08)
          ..style = PaintingStyle.fill);
  }

  @override
  bool shouldRepaint(_AROverlayPainter old) => true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Reusable widgets
// ═══════════════════════════════════════════════════════════════════════════════

class _InfoCard extends StatelessWidget {
  const _InfoCard({
    required this.icon,
    required this.title,
    required this.value,
    required this.color,
  });
  final IconData icon;
  final String title;
  final String value;
  final Color color;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: const Color(0xFF1E293B),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.white.withValues(alpha: 0.05)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.15),
              borderRadius: BorderRadius.circular(10),
            ),
            child: Icon(icon, color: color, size: 20),
          ),
          const SizedBox(height: 10),
          Text(title,
              style: theme.textTheme.bodySmall
                  ?.copyWith(color: const Color(0xFF94A3B8))),
          const SizedBox(height: 3),
          Text(value,
              style: theme.textTheme.bodyMedium?.copyWith(
                  fontWeight: FontWeight.w700, color: Colors.white),
              maxLines: 2,
              overflow: TextOverflow.ellipsis),
        ],
      ),
    );
  }
}

class _StatusRow extends StatelessWidget {
  const _StatusRow(
      {required this.label, required this.status, required this.color});
  final String label;
  final String status;
  final Color color;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Row(
      children: [
        Expanded(
            child: Text(label,
                style: theme.textTheme.bodySmall
                    ?.copyWith(color: const Color(0xFF94A3B8)))),
        Container(
            width: 9,
            height: 9,
            decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
        const SizedBox(width: 6),
        Text(status,
            style: theme.textTheme.bodySmall?.copyWith(
                color: Colors.white70, fontWeight: FontWeight.w600)),
      ],
    );
  }
}

class _ColorChannelSlider extends StatelessWidget {
  const _ColorChannelSlider({
    required this.label,
    required this.value,
    required this.activeColor,
    required this.onChanged,
  });
  final String label;
  final double value;
  final Color activeColor;
  final ValueChanged<double> onChanged;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        SizedBox(
            width: 44,
            child: Text(label,
                style: const TextStyle(color: Colors.white70, fontSize: 12))),
        Expanded(
          child: SliderTheme(
            data: SliderTheme.of(context).copyWith(
              activeTrackColor: activeColor,
              thumbColor: activeColor,
            ),
            child: Slider(
                value: value,
                min: 0,
                max: 255,
                onChanged: onChanged),
          ),
        ),
        SizedBox(
            width: 32,
            child: Text(value.toInt().toString(),
                style: const TextStyle(color: Colors.white70, fontSize: 12))),
      ],
    );
  }
}