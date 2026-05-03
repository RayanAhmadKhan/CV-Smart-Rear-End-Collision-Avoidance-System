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
        scaffoldBackgroundColor: const Color(0xFFF5F8FC),
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
  SafetyState _safetyState = SafetyState.unknown;
  TrackingMode _trackingMode = TrackingMode.anyObject;
  String _sensorStatus = 'Starting';
  String _collisionStatus = 'Idle';
  Color _targetColor = const Color(0xFFE53935);
  double _colorTolerance = 75;
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
    if (controller == null || !controller.value.isInitialized) {
      return;
    }
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

      // Try to start image streaming (not supported on all platforms like Windows web)
      bool supportsStreaming = false;
      try {
        await controller.startImageStream(_processFrame);
        supportsStreaming = true;
      } catch (_) {
        // Frame streaming not supported on this platform
        supportsStreaming = false;
      }

      if (!mounted) {
        await controller.dispose();
        return;
      }

      setState(() {
        _cameraController = controller;
        _supportsFrameProcessing = supportsStreaming;
        _sensorStatus = supportsStreaming 
            ? 'Connected' 
            : 'Connected (preview only)';
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

  void _processFrame(CameraImage image) {
    if (_isProcessingFrame) {
      return;
    }
    _isProcessingFrame = true;

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
        _updateSafetyState(SafetyState.safe);
      }
    } finally {
      _isProcessingFrame = false;
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

    int minX = width;
    int maxX = 0;
    int minY = height;
    int maxY = 0;
    int targetPixels = 0;

    final targetR = _targetColor.red;
    final targetG = _targetColor.green;
    final targetB = _targetColor.blue;
    final maxDistanceSquared = _colorTolerance * _colorTolerance;

    for (int y = 0; y < height; y += 4) {
      for (int x = 0; x < width; x += 4) {
        final yIndex = y * yBytesPerRow + x;
        final uvRow = y ~/ 2;
        final uvCol = x ~/ 2;

        final uIndex = uvRow * uBytesPerRow + uvCol * uBytesPerPixel;
        final vIndex = uvRow * vBytesPerRow + uvCol * vBytesPerPixel;

        if (yIndex >= yPlane.length || uIndex >= uPlane.length || vIndex >= vPlane.length) {
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

  void _updateSafetyState(SafetyState nextState) {
    if (_safetyState == nextState) {
      return;
    }

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
    // Hook for warning sound integration.
    SystemSound.play(SystemSoundType.alert);
    HapticFeedback.mediumImpact();
    if (isDanger) {
      HapticFeedback.heavyImpact();
    }
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

  String _trackingModeLabel() {
    return _trackingMode == TrackingMode.anyObject
        ? 'Any Object'
        : 'Target Color';
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final stateColor = _stateColor();
    final stateLabel = _stateLabel();
    final distanceLabel = !_supportsFrameProcessing
        ? 'N/A'
        : _distanceM == null ? '--.- m' : '${_distanceM!.toStringAsFixed(2)} m';
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
            Row(
              children: [
                Container(
                  width: 48,
                  height: 48,
                  decoration: BoxDecoration(
                    color: const Color(0xFF0E6BA8),
                    borderRadius: BorderRadius.circular(14),
                  ),
                  child: const Icon(
                    Icons.directions_car_rounded,
                    color: Colors.white,
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Rear Distance Monitor',
                        style: theme.textTheme.titleLarge?.copyWith(
                          fontWeight: FontWeight.w700,
                        ),
                      ),
                      Text(
                        'Obstacle awareness dashboard',
                        style: theme.textTheme.bodyMedium?.copyWith(
                          color: const Color(0xFF607086),
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
            const SizedBox(height: 24),
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
                            imageWidth: _cameraController!.value.previewSize?.width ?? 1,
                            imageHeight: _cameraController!.value.previewSize?.height ?? 1,
                          ),
                        ),
                        Align(
                          alignment: Alignment.bottomCenter,
                          child: Container(
                            padding: const EdgeInsets.symmetric(
                              horizontal: 12,
                              vertical: 8,
                            ),
                            margin: const EdgeInsets.all(12),
                            decoration: BoxDecoration(
                              color: Colors.black.withValues(alpha: 0.55),
                              borderRadius: BorderRadius.circular(999),
                            ),
                            child: Text(
                              _supportsFrameProcessing
                                  ? 'Mode: ${_trackingMode == TrackingMode.anyObject ? 'Any object' : 'Target color'}'
                                  : 'Preview only (frame processing unavailable)',
                              style: TextStyle(color: Colors.white),
                            ),
                          ),
                        ),
                      ],
                    )
                  : Center(
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          const Icon(
                            Icons.camera_alt_rounded,
                            size: 48,
                            color: Colors.white38,
                          ),
                          const SizedBox(height: 12),
                          Text(
                            _sensorStatus,
                            style: const TextStyle(
                              color: Colors.white70,
                              fontSize: 14,
                            ),
                            textAlign: TextAlign.center,
                          ),
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
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.white,
                borderRadius: BorderRadius.circular(18),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    'Tracking Configuration',
                    style: theme.textTheme.titleMedium?.copyWith(
                      fontWeight: FontWeight.w700,
                    ),
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
      ),
    );
  }
}

class BoundingBoxPainter extends CustomPainter {
  final List<ObjectDetection> detections;
  final double imageWidth;
  final double imageHeight;

  BoundingBoxPainter({
    required this.detections,
    required this.imageWidth,
    required this.imageHeight,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final scaleX = size.width / imageWidth;
    final scaleY = size.height / imageHeight;

    for (int i = 0; i < detections.length; i++) {
      final detection = detections[i];
      
      final rect = Rect.fromLTRB(
        detection.minX * scaleX,
        detection.minY * scaleY,
        detection.maxX * scaleX,
        detection.maxY * scaleY,
      );

      // Draw border
      canvas.drawRect(
        rect,
        Paint()
          ..color = const Color(0xFF00FF00)
          ..strokeWidth = 3
          ..style = PaintingStyle.stroke,
      );

      // Draw label with object index
      final textPainter = TextPainter(
        text: TextSpan(
          text: 'Object ${i + 1}',
          style: const TextStyle(
            color: Color(0xFF00FF00),
            fontSize: 12,
            fontWeight: FontWeight.bold,
            backgroundColor: Colors.black54,
          ),
        ),
        textDirection: TextDirection.ltr,
      );
      textPainter.layout();
      textPainter.paint(
        canvas,
        Offset(rect.left, rect.top - 20),
      );
    }
  }

  @override
  bool shouldRepaint(BoundingBoxPainter oldDelegate) {
    return oldDelegate.detections.length != detections.length ||
        oldDelegate.imageWidth != imageWidth ||
        oldDelegate.imageHeight != imageHeight;
  }
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
        color: Colors.white,
        borderRadius: BorderRadius.circular(18),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Icon(icon, color: color),
          const SizedBox(height: 8),
          Text(
            title,
            style: theme.textTheme.bodyMedium?.copyWith(
              color: const Color(0xFF607086),
            ),
          ),
          const SizedBox(height: 2),
          Text(
            value,
            style: theme.textTheme.titleMedium?.copyWith(
              fontWeight: FontWeight.w700,
            ),
          ),
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
        Expanded(
          child: Text(
            label,
            style: theme.textTheme.bodyMedium,
          ),
        ),
        Container(
          width: 10,
          height: 10,
          decoration: BoxDecoration(
            color: color,
            shape: BoxShape.circle,
          ),
        ),
        const SizedBox(width: 8),
        Text(
          status,
          style: theme.textTheme.bodyMedium?.copyWith(
            color: const Color(0xFF3D4A59),
            fontWeight: FontWeight.w600,
          ),
        ),
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
          width: 48,
          child: Text(label),
        ),
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
              onChanged: onChanged,
            ),
          ),
        ),
        SizedBox(
          width: 36,
          child: Text(value.toInt().toString()),
        ),
      ],
    );
  }
}
