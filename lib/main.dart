import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:camera/camera.dart';

void main() {
  runApp(const RearCollisionMonitorApp());
}

class RearCollisionMonitorApp extends StatelessWidget {
  const RearCollisionMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    final scheme = ColorScheme.fromSeed(
      seedColor: const Color(0xFF0E6BA8),
      brightness: Brightness.light,
    );

    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Rear Distance Monitor',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: scheme,
        scaffoldBackgroundColor: const Color(0xFF0A0F16),
      ),
      home: const RearDistanceDashboard(),
    );
  }
}

enum SafetyState { safe, warning, danger, unknown }

enum TrackingMode { anyObject, targetColor }

class ObjectDetection {
  final double minX;
  final double maxX;
  final double minY;
  final double maxY;
  final double width;
  final double height;
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

  /// Center of the bounding box
  Offset get center => Offset((minX + maxX) / 2, (minY + maxY) / 2);

  /// Area of the bounding box (proxy for object size)
  double get area => width * height;

  /// Returns how well this detection overlaps with [other] using IoU
  double iou(ObjectDetection other) {
    final interLeft = minX > other.minX ? minX : other.minX;
    final interTop = minY > other.minY ? minY : other.minY;
    final interRight = maxX < other.maxX ? maxX : other.maxX;
    final interBottom = maxY < other.maxY ? maxY : other.maxY;

    final interW = interRight - interLeft;
    final interH = interBottom - interTop;
    if (interW <= 0 || interH <= 0) return 0.0;

    final interArea = interW * interH;
    final unionArea = area + other.area - interArea;
    return unionArea > 0 ? interArea / unionArea : 0.0;
  }
}

/// Helper to smooth jumpy distance values using a moving average
class DistanceSmoother {
  final List<double> _buffer = [];
  final int windowSize;

  DistanceSmoother({this.windowSize = 8});

  double add(double value) {
    _buffer.add(value);
    if (_buffer.length > windowSize) {
      _buffer.removeAt(0);
    }
    return _buffer.reduce((a, b) => a + b) / _buffer.length;
  }

  void clear() => _buffer.clear();
}

class RearDistanceDashboard extends StatefulWidget {
  const RearDistanceDashboard({super.key});

  @override
  State<RearDistanceDashboard> createState() => _RearDistanceDashboardState();
}

class _RearDistanceDashboardState extends State<RearDistanceDashboard>
    with WidgetsBindingObserver, SingleTickerProviderStateMixin {
  static const double _warningThresholdM = 1.8;
  static const double _dangerThresholdM = 0.7;
  static const double _knownObjectWidthM = 0.45;
  static const double _calibratedFocalLength = 0.65;

  // ── Single-object locking ──────────────────────────────────────────────────
  /// The bounding box we are currently locked onto (in image coords).
  ObjectDetection? _lockedDetection;

  /// How many consecutive frames the locked object has been missing.
  int _missingFrameCount = 0;

  /// Release the lock after this many consecutive frames with no match.
  static const int _maxMissingFrames = 8;

  /// Minimum IoU to consider a new detection the "same" object as the locked one.
  static const double _iouMatchThreshold = 0.15;

  /// Maximum centre-distance (in image-width fraction) to match when IoU is low.
  static const double _centreFractionThreshold = 0.30;
  // ──────────────────────────────────────────────────────────────────────────

  CameraController? _cameraController;
  bool _isCameraReady = false;
  bool _isProcessingFrame = false;
  bool _supportsFrameProcessing = false;
  double? _distanceM;
  SafetyState _safetyState = SafetyState.unknown;
  TrackingMode _trackingMode = TrackingMode.anyObject;
  String _sensorStatus = 'Starting';
  String _collisionStatus = 'Idle';
  Color _targetColor = const Color(0xFFE53935);
  double _colorTolerance = 75;
  final DistanceSmoother _distanceSmoother = DistanceSmoother();
  Uint8List? _previousLuma;
  List<ObjectDetection> _detections = [];

  late final AnimationController _dangerPulseController;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _dangerPulseController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 900),
    );
    _initializeCamera();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    final controller = _cameraController;
    if (controller == null || !controller.value.isInitialized) return;
    if (state == AppLifecycleState.inactive ||
        state == AppLifecycleState.paused) {
      controller.stopImageStream();
      controller.dispose();
      _cameraController = null;
      _isCameraReady = false;
    } else if (state == AppLifecycleState.resumed) {
      _initializeCamera();
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _dangerPulseController.dispose();
    _cameraController?.dispose();
    super.dispose();
  }

  Future<void> _initializeCamera() async {
    try {
      final cameras = await availableCameras();
      if (cameras.isEmpty) {
        if (!mounted) return;
        setState(() {
          _sensorStatus = 'No camera found';
          _isCameraReady = false;
        });
        return;
      }

      final rearCamera = cameras.firstWhere(
        (camera) => camera.lensDirection == CameraLensDirection.back,
        orElse: () => cameras.firstWhere(
          (camera) => camera.lensDirection == CameraLensDirection.front,
          orElse: () => cameras.first,
        ),
      );

      final controller = CameraController(
        rearCamera,
        ResolutionPreset.medium,
        enableAudio: false,
        imageFormatGroup: ImageFormatGroup.yuv420,
      );

      await controller.initialize();
      if (!mounted) {
        await controller.dispose();
        return;
      }

      bool supportsStreaming = false;
      try {
        await controller.startImageStream(_processFrame);
        supportsStreaming = true;
      } catch (_) {
        supportsStreaming = false;
      }

      if (!mounted) {
        await controller.dispose();
        return;
      }

      setState(() {
        _cameraController = controller;
        _supportsFrameProcessing = supportsStreaming;
        _sensorStatus =
            supportsStreaming ? 'Connected' : 'Connected (preview only)';
        _isCameraReady = true;
      });
    } catch (e) {
      if (!mounted) return;
      // ignore: avoid_print
      print('Camera initialization error: $e');
      setState(() {
        _sensorStatus = 'Error: ${e.toString().split('\n').first}';
        _isCameraReady = false;
      });
    }
  }

  // ── Core frame processing with single-object locking ──────────────────────

  void _processFrame(CameraImage image) {
    if (_isProcessingFrame) return;
    _isProcessingFrame = true;

    try {
      final candidates = _trackingMode == TrackingMode.anyObject
          ? _detectAnyObject(image)
          : _detectTargetColorObject(image);

      if (!mounted) {
        _isProcessingFrame = false;
        return;
      }

      if (candidates.isEmpty) {
        // No detections at all
        _missingFrameCount++;
        if (_missingFrameCount >= _maxMissingFrames) {
          // Release lock after sustained absence
          _lockedDetection = null;
          _missingFrameCount = 0;
          _distanceSmoother.clear();
          setState(() {
            _distanceM = null;
            _collisionStatus = 'Scanning...';
            _detections = [];
          });
          _updateSafetyState(SafetyState.unknown);
        }
        // else: keep the last locked box visible for a few frames (reduces flicker)
        return;
      }

      // ── Match candidates to the currently locked object ──────────────────
      final imageWidth = image.width.toDouble();
      ObjectDetection? matched;

      if (_lockedDetection == null) {
        // No lock yet — choose the largest (most prominent) candidate
        matched = candidates.reduce(
            (a, b) => a.area >= b.area ? a : b);
      } else {
        // Try to find the best match for the locked object among candidates
        double bestScore = -1.0;
        for (final candidate in candidates) {
          final iou = _lockedDetection!.iou(candidate);
          // Also compute normalised centre distance as a fallback
          final dx =
              (candidate.center.dx - _lockedDetection!.center.dx) / imageWidth;
          final dy =
              (candidate.center.dy - _lockedDetection!.center.dy) / imageWidth;
          final centreDist = (dx * dx + dy * dy);

          // Combine IoU + proximity into a single score
          final score = iou + (1.0 - centreDist.clamp(0.0, 1.0));

          if (score > bestScore) {
            bestScore = score;
            matched = candidate;
          }
        }

        // Accept the match only if it's close enough
        final iouOk = _lockedDetection!.iou(matched!) >= _iouMatchThreshold;
        final dx = (matched.center.dx - _lockedDetection!.center.dx) / imageWidth;
        final dy = (matched.center.dy - _lockedDetection!.center.dy) / imageWidth;
        final centreOk =
            (dx * dx + dy * dy) <= _centreFractionThreshold * _centreFractionThreshold;

        if (!iouOk && !centreOk) {
          // The best candidate is too far from the locked object — keep old lock
          // and increment missing counter
          _missingFrameCount++;
          if (_missingFrameCount >= _maxMissingFrames) {
            // Switch to the new largest object (re-acquire)
            _lockedDetection = candidates.reduce(
                (a, b) => a.area >= b.area ? a : b);
            _missingFrameCount = 0;
            _distanceSmoother.clear();
          }
          return;
        }
      }

      // We have a confirmed matched detection — update lock
      _lockedDetection = matched;
      _missingFrameCount = 0;

      final smoothedDistance = _distanceSmoother.add(matched!.distance);

      // Rebuild the detection with the smoothed distance & correct colour
      final displayDetection = ObjectDetection(
        minX: matched.minX,
        maxX: matched.maxX,
        minY: matched.minY,
        maxY: matched.maxY,
        distance: smoothedDistance,
        color: _getDistanceColor(smoothedDistance),
      );

      setState(() {
        _detections = [displayDetection]; // Always exactly ONE box
        _distanceM = smoothedDistance;
        _collisionStatus = 'Tracking Object';
      });

      if (smoothedDistance < _dangerThresholdM) {
        _updateSafetyState(SafetyState.danger);
      } else if (smoothedDistance < _warningThresholdM) {
        _updateSafetyState(SafetyState.warning);
      } else {
        _updateSafetyState(SafetyState.safe);
      }
    } finally {
      _isProcessingFrame = false;
    }
  }

  // ──────────────────────────────────────────────────────────────────────────

  List<ObjectDetection> _detectAnyObject(CameraImage image) {
    final width = image.width;
    final height = image.height;
    final yPlane = image.planes[0].bytes;
    final yBytesPerRow = image.planes[0].bytesPerRow;

    final previous = _previousLuma;
    if (previous == null || previous.length != yPlane.length) {
      _previousLuma = Uint8List.fromList(yPlane);
      return [];
    }

    const int gridSize = 20;
    final int cols = width ~/ gridSize;
    final int rows = height ~/ gridSize;
    final List<bool> activeGrid = List.filled(cols * rows, false);

    const int motionThreshold = 25;
    bool motionFound = false;

    for (int y = 0; y < height; y += 8) {
      for (int x = 0; x < width; x += 8) {
        final index = y * yBytesPerRow + x;
        if (index < yPlane.length && index < previous.length) {
          final diff = (yPlane[index] - previous[index]).abs();
          if (diff > motionThreshold) {
            final gridX = x ~/ gridSize;
            final gridY = y ~/ gridSize;
            activeGrid[gridY * cols + gridX] = true;
            motionFound = true;
          }
        }
      }
    }

    _previousLuma = Uint8List.fromList(yPlane);
    if (!motionFound) return [];

    final List<ObjectDetection> detections = [];
    final List<bool> visited = List.filled(cols * rows, false);

    for (int i = 0; i < activeGrid.length; i++) {
      if (activeGrid[i] && !visited[i]) {
        int minGX = i % cols,
            maxGX = i % cols,
            minGY = i ~/ cols,
            maxGY = i ~/ cols;

        final List<int> queue = [i];
        visited[i] = true;
        int clusterSize = 0;

        while (queue.isNotEmpty && clusterSize < 50) {
          final current = queue.removeAt(0);
          clusterSize++;
          final cx = current % cols;
          final cy = (current / cols).floor();

          if (cx < minGX) minGX = cx;
          if (cx > maxGX) maxGX = cx;
          if (cy < minGY) minGY = cy;
          if (cy > maxGY) maxGY = cy;

          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              final nx = cx + dx;
              final ny = cy + dy;
              if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                final nIdx = ny * cols + nx;
                if (activeGrid[nIdx] && !visited[nIdx]) {
                  visited[nIdx] = true;
                  queue.add(nIdx);
                }
              }
            }
          }
        }

        if (clusterSize > 3) {
          final minX = minGX * gridSize.toDouble();
          final maxX = (maxGX + 1) * gridSize.toDouble();
          final minY = minGY * gridSize.toDouble();
          final maxY = (maxGY + 1) * gridSize.toDouble();

          final objWidthRatio = (maxX - minX) / width;
          final distance =
              (_knownObjectWidthM * _calibratedFocalLength) / objWidthRatio;
          final clampedDist = distance.clamp(0.2, 8.0);

          detections.add(ObjectDetection(
            minX: minX,
            maxX: maxX,
            minY: minY,
            maxY: maxY,
            distance: clampedDist,
            color: _getDistanceColor(clampedDist),
          ));
        }
      }
    }

    return detections;
  }

  Color _getDistanceColor(double distance) {
    if (distance < _dangerThresholdM) return const Color(0xFFC23A22);
    if (distance < _warningThresholdM) return const Color(0xFFD47D00);
    return const Color(0xFF1D8A4A);
  }

  List<ObjectDetection> _detectTargetColorObject(CameraImage image) {
    final width = image.width;
    final height = image.height;
    final yPlane = image.planes[0].bytes;
    final uPlane = image.planes[1].bytes;
    final vPlane = image.planes[2].bytes;
    final yBytesPerRow = image.planes[0].bytesPerRow;
    final uBytesPerRow = image.planes[1].bytesPerRow;
    final uBytesPerPixel = image.planes[1].bytesPerPixel ?? 1;
    final vBytesPerRow = image.planes[2].bytesPerRow;
    final vBytesPerPixel = image.planes[2].bytesPerPixel ?? 1;

    const int gridSize = 16;
    final int cols = width ~/ gridSize;
    final int rows = height ~/ gridSize;
    final List<bool> activeGrid = List.filled(cols * rows, false);

    final targetR = _targetColor.red;
    final targetG = _targetColor.green;
    final targetB = _targetColor.blue;
    final maxDistSq = _colorTolerance * _colorTolerance;

    for (int gy = 0; gy < rows; gy++) {
      for (int gx = 0; gx < cols; gx++) {
        final x = gx * gridSize + (gridSize ~/ 2);
        final y = gy * gridSize + (gridSize ~/ 2);

        final yIndex = y * yBytesPerRow + x;
        final uvRow = y ~/ 2;
        final uvCol = x ~/ 2;
        final uIndex = uvRow * uBytesPerRow + uvCol * uBytesPerPixel;
        final vIndex = uvRow * vBytesPerRow + uvCol * vBytesPerPixel;

        if (yIndex < yPlane.length &&
            uIndex < uPlane.length &&
            vIndex < vPlane.length) {
          final yVal = yPlane[yIndex].toDouble();
          final uVal = uPlane[uIndex].toDouble() - 128.0;
          final vVal = vPlane[vIndex].toDouble() - 128.0;

          final r = (yVal + 1.402 * vVal).clamp(0, 255);
          final g =
              (yVal - 0.344136 * uVal - 0.714136 * vVal).clamp(0, 255);
          final b = (yVal + 1.772 * uVal).clamp(0, 255);

          final distSq = (r - targetR) * (r - targetR) +
              (g - targetG) * (g - targetG) +
              (b - targetB) * (b - targetB);
          if (distSq < maxDistSq) {
            activeGrid[gy * cols + gx] = true;
          }
        }
      }
    }

    final List<ObjectDetection> detections = [];
    final List<bool> visited = List.filled(cols * rows, false);

    for (int i = 0; i < activeGrid.length; i++) {
      if (activeGrid[i] && !visited[i]) {
        int minGX = i % cols,
            maxGX = i % cols,
            minGY = i ~/ cols,
            maxGY = i ~/ cols;
        final List<int> queue = [i];
        visited[i] = true;
        int size = 0;

        while (queue.isNotEmpty && size < 40) {
          final c = queue.removeAt(0);
          size++;
          final cx = c % cols, cy = c ~/ cols;
          if (cx < minGX) minGX = cx;
          if (cx > maxGX) maxGX = cx;
          if (cy < minGY) minGY = cy;
          if (cy > maxGY) maxGY = cy;

          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              final nx = cx + dx, ny = cy + dy;
              if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                final nIdx = ny * cols + nx;
                if (activeGrid[nIdx] && !visited[nIdx]) {
                  visited[nIdx] = true;
                  queue.add(nIdx);
                }
              }
            }
          }
        }

        if (size > 2) {
          final minX = minGX * gridSize.toDouble(),
              maxX = (maxGX + 1) * gridSize.toDouble();
          final minY = minGY * gridSize.toDouble(),
              maxY = (maxGY + 1) * gridSize.toDouble();
          final ratio = (maxX - minX) / width;
          final dist =
              ((_knownObjectWidthM * _calibratedFocalLength) / ratio)
                  .clamp(0.2, 8.0);

          detections.add(ObjectDetection(
            minX: minX,
            maxX: maxX,
            minY: minY,
            maxY: maxY,
            distance: dist,
            color: _getDistanceColor(dist),
          ));
        }
      }
    }
    return detections;
  }

  void _updateSafetyState(SafetyState nextState) {
    if (_safetyState == nextState) return;

    final previousState = _safetyState;
    setState(() {
      _safetyState = nextState;
    });

    if (nextState == SafetyState.danger) {
      _dangerPulseController.repeat(reverse: true);
      _triggerWarningSound(isDanger: true);
    } else {
      _dangerPulseController.stop();
      _dangerPulseController.reset();
      if (nextState == SafetyState.warning && previousState != nextState) {
        _triggerWarningSound(isDanger: false);
      }
    }
  }

  void _triggerWarningSound({required bool isDanger}) {
    SystemSound.play(SystemSoundType.alert);
    HapticFeedback.mediumImpact();
    if (isDanger) HapticFeedback.heavyImpact();
  }

  /// Resets the object lock manually (e.g. when user switches tracking mode)
  void _resetLock() {
    _lockedDetection = null;
    _missingFrameCount = 0;
    _distanceSmoother.clear();
    _previousLuma = null;
    _detections = [];
    _distanceM = null;
    _collisionStatus = 'Scanning...';
  }

  Color _stateColor() {
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

  String _stateLabel() {
    switch (_safetyState) {
      case SafetyState.safe:
        return 'Safe Distance';
      case SafetyState.warning:
        return 'Warning Zone';
      case SafetyState.danger:
        return 'Danger Zone';
      case SafetyState.unknown:
        return 'Searching';
    }
  }

  String _trackingModeLabel() =>
      _trackingMode == TrackingMode.anyObject ? 'Any Object' : 'Target Color';

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final stateColor = _stateColor();
    final stateLabel = _stateLabel();
    final distanceLabel = !_supportsFrameProcessing
        ? 'N/A'
        : _distanceM == null
            ? '--.- m'
            : '${_distanceM!.toStringAsFixed(2)} m';
    final isDanger = _safetyState == SafetyState.danger;

    final badgeColor = switch (_safetyState) {
      SafetyState.safe => const Color(0xFF1D8A4A),
      SafetyState.warning => const Color(0xFFD47D00),
      SafetyState.danger => const Color(0xFFC23A22),
      SafetyState.unknown => const Color(0xFF607086),
    };

    return Scaffold(
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.fromLTRB(20, 16, 20, 24),
          children: [
            // ── Header ──────────────────────────────────────────────────────
            Row(
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
                        color: const Color(0xFF0E6BA8).withValues(alpha: 0.3),
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
                        'Single-Object Focus & Distance Detection',
                        style: theme.textTheme.bodyMedium
                            ?.copyWith(color: const Color(0xFF94A3B8)),
                      ),
                    ],
                  ),
                ),
              ],
            ),

            const SizedBox(height: 24),

            // ── Camera Preview ───────────────────────────────────────────────
            Container(
              height: 210,
              decoration: BoxDecoration(
                color: Colors.black,
                borderRadius: BorderRadius.circular(20),
              ),
              clipBehavior: Clip.antiAlias,
              child: _isCameraReady && _cameraController != null
                  ? Stack(
                      fit: StackFit.expand,
                      children: [
                        CameraPreview(_cameraController!),
                        CustomPaint(
                          painter: BoundingBoxPainter(
                            detections: _detections,
                            imageWidth: _cameraController!
                                    .value.previewSize?.width ??
                                1,
                            imageHeight: _cameraController!
                                    .value.previewSize?.height ??
                                1,
                            isLocked: _lockedDetection != null,
                          ),
                        ),
                        // Lock indicator badge
                        if (_lockedDetection != null)
                          Positioned(
                            top: 10,
                            right: 10,
                            child: Container(
                              padding: const EdgeInsets.symmetric(
                                  horizontal: 10, vertical: 5),
                              decoration: BoxDecoration(
                                color: const Color(0xFF0E6BA8)
                                    .withValues(alpha: 0.85),
                                borderRadius: BorderRadius.circular(999),
                              ),
                              child: Row(
                                mainAxisSize: MainAxisSize.min,
                                children: const [
                                  Icon(Icons.lock_rounded,
                                      color: Colors.white, size: 12),
                                  SizedBox(width: 4),
                                  Text('Locked',
                                      style: TextStyle(
                                          color: Colors.white,
                                          fontSize: 11,
                                          fontWeight: FontWeight.w700)),
                                ],
                              ),
                            ),
                          ),
                        // Tap to reset lock
                        Positioned.fill(
                          child: GestureDetector(
                            onTap: () {
                              setState(_resetLock);
                            },
                            behavior: HitTestBehavior.translucent,
                          ),
                        ),
                        Align(
                          alignment: Alignment.bottomCenter,
                          child: Container(
                            padding: const EdgeInsets.symmetric(
                                horizontal: 12, vertical: 8),
                            margin: const EdgeInsets.all(12),
                            decoration: BoxDecoration(
                              color: Colors.black.withValues(alpha: 0.55),
                              borderRadius: BorderRadius.circular(999),
                            ),
                            child: Text(
                              _supportsFrameProcessing
                                  ? (_lockedDetection != null
                                      ? 'Tap to reset lock'
                                      : 'Mode: ${_trackingMode == TrackingMode.anyObject ? "Any object" : "Target color"} • Searching…')
                                  : 'Preview only (frame processing unavailable)',
                              style: const TextStyle(color: Colors.white),
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
                          const SizedBox(height: 16),
                          ElevatedButton.icon(
                            onPressed: _initializeCamera,
                            icon: const Icon(Icons.refresh_rounded),
                            label: const Text('Retry Camera'),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: const Color(0xFF0E6BA8),
                              foregroundColor: Colors.white,
                            ),
                          ),
                        ],
                      ),
                    ),
            ),

            const SizedBox(height: 20),

            // ── Tracking Configuration ───────────────────────────────────────
            Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: const Color(0xFF1E293B),
                borderRadius: BorderRadius.circular(24),
                border:
                    Border.all(color: Colors.white.withValues(alpha: 0.05)),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('Tracking Configuration',
                      style: theme.textTheme.titleMedium
                          ?.copyWith(fontWeight: FontWeight.w700)),
                  const SizedBox(height: 10),
                  if (!_supportsFrameProcessing)
                    Container(
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: const Color(0xFFFFF3CD),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Row(
                        children: [
                          const Icon(Icons.info_rounded,
                              color: Color(0xFF856404), size: 20),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              'Distance detection requires a device that supports frame streaming (phones/tablets).',
                              style: theme.textTheme.bodySmall
                                  ?.copyWith(color: const Color(0xFF856404)),
                            ),
                          ),
                        ],
                      ),
                    )
                  else ...[
                    SegmentedButton<TrackingMode>(
                      segments: const [
                        ButtonSegment(
                          value: TrackingMode.anyObject,
                          label: Text('Any Object'),
                          icon: Icon(Icons.visibility_rounded),
                        ),
                        ButtonSegment(
                          value: TrackingMode.targetColor,
                          label: Text('Target Color'),
                          icon: Icon(Icons.palette_rounded),
                        ),
                      ],
                      selected: {_trackingMode},
                      onSelectionChanged: (selection) {
                        setState(() {
                          _trackingMode = selection.first;
                          _resetLock();
                        });
                      },
                    ),
                    const SizedBox(height: 10),
                    Text(
                      _trackingMode == TrackingMode.anyObject
                          ? 'Locks onto the largest moving object detected.'
                          : 'Locks onto the closest object matching the target color.',
                      style: theme.textTheme.bodyMedium
                          ?.copyWith(color: const Color(0xFF607086)),
                    ),
                  ],
                  if (_trackingMode == TrackingMode.targetColor &&
                      _supportsFrameProcessing) ...[
                    const SizedBox(height: 14),
                    Row(
                      children: [
                        Container(
                          width: 24,
                          height: 24,
                          decoration: BoxDecoration(
                            color: _targetColor,
                            borderRadius: BorderRadius.circular(6),
                            border: Border.all(
                                color: const Color(0xFFCBD5E1)),
                          ),
                        ),
                        const SizedBox(width: 10),
                        Text('Selected Color',
                            style: theme.textTheme.bodyMedium),
                      ],
                    ),
                    const SizedBox(height: 8),
                    _ColorChannelSlider(
                      label: 'Red',
                      value: _targetColor.red.toDouble(),
                      activeColor: Colors.red,
                      onChanged: (v) => setState(
                          () => _targetColor = _targetColor.withRed(v.toInt())),
                    ),
                    _ColorChannelSlider(
                      label: 'Green',
                      value: _targetColor.green.toDouble(),
                      activeColor: Colors.green,
                      onChanged: (v) => setState(() =>
                          _targetColor = _targetColor.withGreen(v.toInt())),
                    ),
                    _ColorChannelSlider(
                      label: 'Blue',
                      value: _targetColor.blue.toDouble(),
                      activeColor: Colors.blue,
                      onChanged: (v) => setState(() =>
                          _targetColor = _targetColor.withBlue(v.toInt())),
                    ),
                    const SizedBox(height: 4),
                    Text('Tolerance: ${_colorTolerance.toStringAsFixed(0)}',
                        style: theme.textTheme.bodySmall),
                    Slider(
                      value: _colorTolerance,
                      min: 20,
                      max: 140,
                      onChanged: (v) =>
                          setState(() => _colorTolerance = v),
                    ),
                  ],
                ],
              ),
            ),

            const SizedBox(height: 20),

            // ── Distance Display ─────────────────────────────────────────────
            Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                gradient: LinearGradient(
                  colors: [stateColor.withValues(alpha: 0.9), stateColor],
                  begin: Alignment.topLeft,
                  end: Alignment.bottomRight,
                ),
                borderRadius: BorderRadius.circular(24),
                boxShadow: [
                  BoxShadow(
                    color: stateColor.withValues(alpha: 0.22),
                    blurRadius: 18,
                    offset: const Offset(0, 10),
                  ),
                ],
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      Text(
                        'Current Rear Distance',
                        style: theme.textTheme.bodyLarge?.copyWith(
                          color: Colors.white.withValues(alpha: 0.92),
                          fontWeight: FontWeight.w500,
                        ),
                      ),
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 10, vertical: 6),
                        decoration: BoxDecoration(
                          color: Colors.white.withValues(alpha: 0.2),
                          borderRadius: BorderRadius.circular(999),
                        ),
                        child: Text(
                          '$stateLabel • ${_trackingModeLabel()}',
                          style: const TextStyle(
                              color: Colors.white,
                              fontWeight: FontWeight.w600,
                              fontSize: 12),
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 14),
                  Text(
                    distanceLabel,
                    style: theme.textTheme.displayMedium?.copyWith(
                      color: Colors.white,
                      fontWeight: FontWeight.w700,
                      letterSpacing: 1.2,
                    ),
                  ),
                  const SizedBox(height: 6),
                  Text(
                    !_supportsFrameProcessing
                        ? 'Frame processing not available on this platform'
                        : _distanceM == null
                            ? 'Searching for object to lock onto'
                            : 'Locked — estimated from object size in frame',
                    style: theme.textTheme.bodyMedium?.copyWith(
                        color: Colors.white.withValues(alpha: 0.85)),
                  ),
                ],
              ),
            ),

            const SizedBox(height: 20),

            Row(
              children: [
                Expanded(
                  child: _InfoCard(
                    icon: Icons.warning_amber_rounded,
                    title: 'Alert Zone',
                    value: '< ${_dangerThresholdM.toStringAsFixed(1)} m',
                    color: badgeColor,
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: _InfoCard(
                    icon: Icons.speed_rounded,
                    title: 'Brake Hint',
                    value: _safetyState == SafetyState.danger
                        ? 'Brake Now'
                        : _safetyState == SafetyState.warning
                            ? 'Slow Down'
                            : 'Standby',
                    color: _safetyState == SafetyState.danger
                        ? const Color(0xFFC23A22)
                        : _safetyState == SafetyState.warning
                            ? const Color(0xFFD47D00)
                            : const Color(0xFF1D8A4A),
                  ),
                ),
              ],
            ),

            if (isDanger) ...[
              const SizedBox(height: 16),
              Center(
                child: AnimatedBuilder(
                  animation: _dangerPulseController,
                  builder: (context, child) {
                    final t = _dangerPulseController.value;
                    return Container(
                      width: 78 + (16 * t),
                      height: 78 + (16 * t),
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: const Color(0xFFC23A22)
                            .withValues(alpha: 0.12 + (0.12 * (1 - t))),
                      ),
                      child: Center(
                        child: Container(
                          width: 64,
                          height: 64,
                          decoration: const BoxDecoration(
                            color: Color(0xFFC23A22),
                            shape: BoxShape.circle,
                          ),
                          child: const Icon(Icons.priority_high_rounded,
                              color: Colors.white, size: 34),
                        ),
                      ),
                    );
                  },
                ),
              ),
            ],

            const SizedBox(height: 20),

            // ── Status Panel ─────────────────────────────────────────────────
            Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: const Color(0xFF1E293B),
                borderRadius: BorderRadius.circular(24),
                border:
                    Border.all(color: Colors.white.withValues(alpha: 0.05)),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('Rear View Status',
                      style: theme.textTheme.titleMedium
                          ?.copyWith(fontWeight: FontWeight.w700)),
                  const SizedBox(height: 12),
                  _StatusRow(
                    label: 'Sensor Connection',
                    status: _sensorStatus,
                    color: _isCameraReady
                        ? const Color(0xFF1D8A4A)
                        : const Color(0xFFC23A22),
                  ),
                  const SizedBox(height: 8),
                  _StatusRow(
                    label: 'Object Lock',
                    status: _lockedDetection != null ? 'Locked' : 'Searching',
                    color: _lockedDetection != null
                        ? const Color(0xFF0E6BA8)
                        : const Color(0xFF607086),
                  ),
                  const SizedBox(height: 8),
                  _StatusRow(
                    label: 'Collision Prediction',
                    status: !_supportsFrameProcessing
                        ? 'Unavailable'
                        : _collisionStatus,
                    color: !_supportsFrameProcessing
                        ? const Color(0xFFC23A22)
                        : _safetyState == SafetyState.danger
                            ? const Color(0xFFC23A22)
                            : _safetyState == SafetyState.warning
                                ? const Color(0xFFD47D00)
                                : const Color(0xFF607086),
                  ),
                  const SizedBox(height: 8),
                  _StatusRow(
                    label: 'Frame Processing',
                    status:
                        _supportsFrameProcessing ? 'Active' : 'Not supported',
                    color: _supportsFrameProcessing
                        ? const Color(0xFF1D8A4A)
                        : const Color(0xFFC23A22),
                  ),
                  const SizedBox(height: 8),
                  _StatusRow(
                    label: 'Tracking Mode',
                    status: _trackingModeLabel(),
                    color: const Color(0xFF0E6BA8),
                  ),
                  const SizedBox(height: 8),
                  _StatusRow(
                    label: 'Audio Warning',
                    status: _safetyState == SafetyState.danger
                        ? 'Triggered'
                        : _safetyState == SafetyState.warning
                            ? 'Armed'
                            : 'Ready',
                    color: _safetyState == SafetyState.danger
                        ? const Color(0xFFC23A22)
                        : const Color(0xFF1D8A4A),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// ── BoundingBoxPainter ───────────────────────────────────────────────────────

class BoundingBoxPainter extends CustomPainter {
  final List<ObjectDetection> detections;
  final double imageWidth;
  final double imageHeight;
  final bool isLocked;

  BoundingBoxPainter({
    required this.detections,
    required this.imageWidth,
    required this.imageHeight,
    this.isLocked = false,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (imageWidth == 0 || imageHeight == 0) return;

    final scaleX = size.width / imageWidth;
    final scaleY = size.height / imageHeight;

    _drawDistanceLines(canvas, size);

    for (final detection in detections) {
      final rect = Rect.fromLTRB(
        detection.minX * scaleX,
        detection.minY * scaleY,
        detection.maxX * scaleX,
        detection.maxY * scaleY,
      );

      final paint = Paint()
        ..color = detection.color
        ..strokeWidth = isLocked ? 4.0 : 3.0
        ..style = PaintingStyle.stroke;

      _drawCorners(canvas, rect, paint);

      // Lock crosshair at centre when locked
      if (isLocked) {
        final cx = (rect.left + rect.right) / 2;
        final cy = (rect.top + rect.bottom) / 2;
        final crossPaint = Paint()
          ..color = detection.color.withValues(alpha: 0.8)
          ..strokeWidth = 1.5
          ..style = PaintingStyle.stroke;
        canvas.drawLine(Offset(cx - 8, cy), Offset(cx + 8, cy), crossPaint);
        canvas.drawLine(Offset(cx, cy - 8), Offset(cx, cy + 8), crossPaint);
      }

      final textPainter = TextPainter(
        text: TextSpan(
          text: '${detection.distance.toStringAsFixed(1)}m',
          style: TextStyle(
            color: Colors.white,
            fontSize: 14,
            fontWeight: FontWeight.bold,
            backgroundColor: detection.color.withValues(alpha: 0.8),
          ),
        ),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(canvas, Offset(rect.left, rect.top - 22));
    }
  }

  void _drawDistanceLines(Canvas canvas, Size size) {
    final linePaint = Paint()
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    final levels = [
      {'dist': '0.5m', 'color': const Color(0xFFC23A22), 'y': 0.85},
      {'dist': '1.0m', 'color': const Color(0xFFD47D00), 'y': 0.65},
      {'dist': '1.5m', 'color': const Color(0xFF1D8A4A), 'y': 0.45},
    ];

    for (final level in levels) {
      linePaint.color =
          (level['color'] as Color).withValues(alpha: 0.4);
      final y = size.height * (level['y'] as double);

      final path = Path();
      path.moveTo(size.width * 0.1, y);
      path.quadraticBezierTo(size.width * 0.5, y + 20, size.width * 0.9, y);
      canvas.drawPath(path, linePaint);

      final tp = TextPainter(
        text: TextSpan(
          text: level['dist'] as String,
          style: TextStyle(
              color: linePaint.color,
              fontSize: 10,
              fontWeight: FontWeight.bold),
        ),
        textDirection: TextDirection.ltr,
      );
      tp.layout();
      tp.paint(canvas, Offset(size.width * 0.05, y - 12));
    }
  }

  void _drawCorners(Canvas canvas, Rect rect, Paint paint) {
    const double len = 15;
    canvas.drawLine(rect.topLeft, rect.topLeft + const Offset(len, 0), paint);
    canvas.drawLine(rect.topLeft, rect.topLeft + const Offset(0, len), paint);
    canvas.drawLine(
        rect.topRight, rect.topRight + const Offset(-len, 0), paint);
    canvas.drawLine(
        rect.topRight, rect.topRight + const Offset(0, len), paint);
    canvas.drawLine(
        rect.bottomLeft, rect.bottomLeft + const Offset(len, 0), paint);
    canvas.drawLine(
        rect.bottomLeft, rect.bottomLeft + const Offset(0, -len), paint);
    canvas.drawLine(
        rect.bottomRight, rect.bottomRight + const Offset(-len, 0), paint);
    canvas.drawLine(
        rect.bottomRight, rect.bottomRight + const Offset(0, -len), paint);

    canvas.drawRect(
        rect,
        paint
          ..strokeWidth = 1
          ..color = paint.color.withValues(alpha: 0.2));
  }

  @override
  bool shouldRepaint(BoundingBoxPainter oldDelegate) => true;
}

// ── Reusable widgets ─────────────────────────────────────────────────────────

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
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(
        color: const Color(0xFF1E293B),
        borderRadius: BorderRadius.circular(22),
        border: Border.all(color: Colors.white.withValues(alpha: 0.05)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.15),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(icon, color: color, size: 22),
          ),
          const SizedBox(height: 12),
          Text(title,
              style: theme.textTheme.bodyMedium
                  ?.copyWith(color: const Color(0xFF94A3B8))),
          const SizedBox(height: 4),
          Text(value,
              style: theme.textTheme.titleLarge?.copyWith(
                  fontWeight: FontWeight.w800, color: Colors.white)),
        ],
      ),
    );
  }
}

class _StatusRow extends StatelessWidget {
  const _StatusRow({
    required this.label,
    required this.status,
    required this.color,
  });

  final String label;
  final String status;
  final Color color;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Row(
      children: [
        Expanded(child: Text(label, style: theme.textTheme.bodyMedium)),
        Container(
            width: 10,
            height: 10,
            decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
        const SizedBox(width: 8),
        Text(status,
            style: theme.textTheme.bodyMedium?.copyWith(
                color: const Color(0xFF3D4A59), fontWeight: FontWeight.w600)),
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
        SizedBox(width: 48, child: Text(label)),
        Expanded(
          child: SliderTheme(
            data: SliderTheme.of(context).copyWith(
              activeTrackColor: activeColor,
              thumbColor: activeColor,
            ),
            child: Slider(value: value, min: 0, max: 255, onChanged: onChanged),
          ),
        ),
        SizedBox(width: 36, child: Text(value.toInt().toString())),
      ],
    );
  }
}