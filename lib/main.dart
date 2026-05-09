import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:camera/camera.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(const RearCollisionMonitorApp());
}


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
  }) : width = (maxX - minX).abs();

  double get height => (maxY - minY).abs();
}

class RearDistanceDashboard extends StatefulWidget {
  const RearDistanceDashboard({super.key});

  @override
  State<RearDistanceDashboard> createState() => _RearDistanceDashboardState();
}

class _RearDistanceDashboardState extends State<RearDistanceDashboard>
    with WidgetsBindingObserver, SingleTickerProviderStateMixin {
  static const double _warningThresholdM = 1.5;
  static const double _dangerThresholdM = 0.8;
  static const double _knownObjectWidthM = 0.5;
  static const double _calibratedFocalLength = 0.62;

  CameraController? _cameraController;
  bool _isCameraReady = false;
  bool _isProcessingFrame = false;
  bool _supportsFrameProcessing = false;
  double? _distanceM;
  bool _brakeApplied = false;
  SafetyState _safetyState = SafetyState.unknown;
  TrackingMode _trackingMode = TrackingMode.anyObject;
  String _sensorStatus = 'Starting';
  String _collisionStatus = 'Idle';
  Color _targetColor = const Color(0xFFE53935);
  double _colorTolerance = 75;
  List<ObjectDetection> _detections = [];

  late final AnimationController _dangerPulseController;

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

// camera initialization
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
        ResolutionPreset.medium, // ~1280×720 
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

//frame processing and detection
  void _onFrame(CameraImage img) {
    if (_processingFrame) return;
    _processingFrame = true;
    try {
      final detections = _trackingMode == TrackingMode.anyObject
          ? _detectAnyObject(image)
          : _detectTargetColorObject(image);

      if (!mounted) {
        _isProcessingFrame = false;
        return;
      }

      if (detections.isEmpty) {
        setState(() {
          _distanceM = null;
          _collisionStatus = 'No target';
          _detections = [];
        });
        _updateSafetyState(SafetyState.unknown);
        return;
      }

      setState(() {
        _detections = detections;
      });

      // Calculate distances for all detections
      final distances = detections.map((detection) {
        final objectWidthRatio = detection.width / image.width;
        return (_knownObjectWidthM * _calibratedFocalLength) / objectWidthRatio;
      }).toList();

      // Find nearest object
      final nearestDistance = distances.reduce((a, b) => a < b ? a : b);
      final clampedDistance = nearestDistance.clamp(0.2, 10.0);

      setState(() {
        _distanceM = clampedDistance;
        _collisionStatus = _trackingMode == TrackingMode.anyObject
            ? 'Tracking ${detections.length} object${detections.length > 1 ? 's' : ''}'
            : 'Tracking ${detections.length} object${detections.length > 1 ? 's' : ''}';
      });

      if (clampedDistance < _dangerThresholdM) {
        _updateSafetyState(SafetyState.danger);
      } else if (clampedDistance < _warningThresholdM) {
        _updateSafetyState(SafetyState.warning);
      } else {
        _setSafety(SafetyState.safe);
      }

      // Persist model periodically (every ~30 frames ≈ 1s)
      if (_sizeModel.sampleCount % 30 == 0 && _sizeModel.sampleCount > 0) _saveModel();
    } finally {
      _processingFrame = false;
    }
  }

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

    // Motion detection
    int minX = width;
    int maxX = 0;
    int minY = height;
    int maxY = 0;
    int motionPixels = 0;

    const int motionThreshold = 20;

    for (int y = 0; y < height; y += 4) {
      for (int x = 0; x < width; x += 4) {
        final index = y * yBytesPerRow + x;
        if (index < yPlane.length && index < previous.length) {
          final diff = (yPlane[index] - previous[index]).abs();
          if (diff > motionThreshold) {
            motionPixels++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
          }
        }
      }
    }

    _previousLuma = Uint8List.fromList(yPlane);

    // If motion detected
    if (motionPixels > 20 && maxX > minX && maxY > minY) {
      return [ObjectDetection(
        minX: minX.toDouble(),
        maxX: maxX.toDouble(),
        minY: minY.toDouble(),
        maxY: maxY.toDouble(),
      )];
    }

    // Strong edge detection for static objects using Sobel-like approach
    minX = width;
    maxX = 0;
    minY = height;
    maxY = 0;
    int edgePixels = 0;

    const int step = 3;
    const int strongEdgeThreshold = 40;

    for (int y = step; y < height - step; y += step) {
      for (int x = step; x < width - step; x += step) {
        final idx = y * yBytesPerRow + x;
        if (idx + step < yPlane.length) {
          final center = yPlane[idx];
          
          // Sobel-like gradients
          final gx = (yPlane[idx + step] - yPlane[idx - step]).abs();
          final gy = (yPlane[idx + (step * yBytesPerRow)] - yPlane[idx - (step * yBytesPerRow)]).abs();
          
          final edgeStrength = (gx + gy) ~/ 2;
          
          // Only detect strong edges
          if (edgeStrength > strongEdgeThreshold) {
            edgePixels++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
          }
        }
      }
    }

    // Require significant edge regions
    if (edgePixels > 40 && maxX > minX && maxY > minY) {
      final widthPx = maxX - minX;
      final heightPx = maxY - minY;
      
      // Filter out very thin or elongated detections (likely noise)
      if (widthPx > 20 && heightPx > 20) {
        return [ObjectDetection(
          minX: minX.toDouble(),
          maxX: maxX.toDouble(),
          minY: minY.toDouble(),
          maxY: maxY.toDouble(),
        )];
      }
    }

    return [];
  }


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

    int minX = width;
    int maxX = 0;
    int minY = height;
    int maxY = 0;
    int targetPixels = 0;

    final tR = ((_targetColor.value >> 16) & 0xFF).toDouble();
    final tG = ((_targetColor.value >> 8) & 0xFF).toDouble();
    final tB = (_targetColor.value & 0xFF).toDouble();
    final maxDsq = _colorTolerance * _colorTolerance;

    for (int y = 0; y < height; y += 4) {
      for (int x = 0; x < width; x += 4) {
        final yIndex = y * yBytesPerRow + x;
        final uvRow = y ~/ 2;
        final uvCol = x ~/ 2;
        final uIdx = uvRow * uBpr + uvCol * uBpp;
        final vIdx = uvRow * vBpr + uvCol * vBpp;
        if (yIdx >= yP.length || uIdx >= uP.length || vIdx >= vP.length) {
          continue;
        }

        final yValue = yPlane[yIndex].toDouble();
        final uValue = uPlane[uIndex].toDouble() - 128.0;
        final vValue = vPlane[vIndex].toDouble() - 128.0;

        final r = (yValue + 1.402 * vValue).clamp(0, 255).toInt();
        final g = (yValue - 0.344136 * uValue - 0.714136 * vValue)
            .clamp(0, 255)
            .toInt();
        final b = (yValue + 1.772 * uValue).clamp(0, 255).toInt();

        final dr = (r - targetR).toDouble();
        final dg = (g - targetG).toDouble();
        final db = (b - targetB).toDouble();
        final colorDistanceSquared = (dr * dr) + (dg * dg) + (db * db);

        if (colorDistanceSquared <= maxDistanceSquared) {
          targetPixels++;
          if (x < minX) minX = x;
          if (x > maxX) maxX = x;
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
      }
    }

    if (targetPixels > 15 && maxX > minX && maxY > minY) {
      return [ObjectDetection(
        minX: minX.toDouble(),
        maxX: maxX.toDouble(),
        minY: minY.toDouble(),
        maxY: maxY.toDouble(),
      )];
    }

    return [];
  }

  List<ObjectDetection> _clusterObjects(
    List<bool> pixelMap,
    int width,
    int height,
    int minPixels,
  ) {
    // This function is no longer used but kept for compatibility
    return [];
  }


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
                          const Icon(
                            Icons.info_rounded,
                            color: Color(0xFF856404),
                            size: 20,
                          ),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              'Distance detection requires a device that supports frame streaming (phones/tablets).',
                              style: theme.textTheme.bodySmall?.copyWith(
                                color: const Color(0xFF856404),
                              ),
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
                        final selectedMode = selection.first;
                        setState(() {
                          _trackingMode = selectedMode;
                          _distanceM = null;
                          _collisionStatus = 'Mode switched';
                          _previousLuma = null;
                          _detections = [];
                        });
                      },
                    ),
                    const SizedBox(height: 10),
                    Text(
                      _trackingMode == TrackingMode.anyObject
                          ? 'Tracks moving objects in rear camera view.'
                          : 'Tracks objects close to selected color.',
                      style: theme.textTheme.bodyMedium?.copyWith(
                        color: const Color(0xFF607086),
                      ),
                    ),
                  ],
                  if (_trackingMode == TrackingMode.targetColor && _supportsFrameProcessing) ...[
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
                              color: const Color(0xFFCBD5E1),
                            ),
                          ),
                        ),
                        const SizedBox(width: 10),
                        Text(
                          'Selected Color',
                          style: theme.textTheme.bodyMedium,
                        ),
                      ],
                    ),
                    const SizedBox(height: 8),
                    _ColorChannelSlider(
                      label: 'Red',
                      value: _targetColor.red.toDouble(),
                      activeColor: Colors.red,
                      onChanged: (value) {
                        setState(() {
                          _targetColor = _targetColor.withRed(value.toInt());
                        });
                      },
                    ),
                    _ColorChannelSlider(
                      label: 'Green',
                      value: _targetColor.green.toDouble(),
                      activeColor: Colors.green,
                      onChanged: (value) {
                        setState(() {
                          _targetColor = _targetColor.withGreen(value.toInt());
                        });
                      },
                    ),
                    _ColorChannelSlider(
                      label: 'Blue',
                      value: _targetColor.blue.toDouble(),
                      activeColor: Colors.blue,
                      onChanged: (value) {
                        setState(() {
                          _targetColor = _targetColor.withBlue(value.toInt());
                        });
                      },
                    ),
                    const SizedBox(height: 4),
                    Text(
                      'Tolerance: ${_colorTolerance.toStringAsFixed(0)}',
                      style: theme.textTheme.bodySmall,
                    ),
                    Slider(
                      value: _colorTolerance,
                      min: 20,
                      max: 140,
                      onChanged: (value) {
                        setState(() {
                          _colorTolerance = value;
                        });
                      },
                    ),
                  ],
                ],
              ),
            ),
            const SizedBox(height: 20),
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
                          horizontal: 10,
                          vertical: 6,
                        ),
                        decoration: BoxDecoration(
                          color: Colors.white.withValues(alpha: 0.2),
                          borderRadius: BorderRadius.circular(999),
                        ),
                        child: Text(
                          '$stateLabel • ${_trackingModeLabel()}',
                          style: const TextStyle(
                            color: Colors.white,
                            fontWeight: FontWeight.w600,
                            fontSize: 12,
                          ),
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
                            ? 'Searching for tracked object'
                            : 'Estimated from object size in frame',
                    style: theme.textTheme.bodyMedium?.copyWith(
                      color: Colors.white.withValues(alpha: 0.85),
                    ),
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
                          child: const Icon(
                            Icons.priority_high_rounded,
                            color: Colors.white,
                            size: 34,
                          ),
                        ),
                      ),
                    );
                  },
                ),
              ),
            ],
            const SizedBox(height: 20),
            Container(
              padding: const EdgeInsets.all(18),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(20),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Rear View Status',
                    style: theme.textTheme.titleMedium?.copyWith(
                      fontWeight: FontWeight.w700,
                    ),
                  ),
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
                    status: _supportsFrameProcessing ? 'Active' : 'Not supported',
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
      );
}


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