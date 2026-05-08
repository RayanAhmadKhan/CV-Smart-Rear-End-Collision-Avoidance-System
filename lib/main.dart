import 'dart:math' as math;

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
  double get area => width * height;
  Offset get center => Offset((minX + maxX) * 0.5, (minY + maxY) * 0.5);

  double iou(ObjectDetection other) {
    final left = math.max(minX, other.minX);
    final top = math.max(minY, other.minY);
    final right = math.min(maxX, other.maxX);
    final bottom = math.min(maxY, other.maxY);
    if (right <= left || bottom <= top) return 0.0;
    final inter = (right - left) * (bottom - top);
    final union = area + other.area - inter;
    return union > 0 ? inter / union : 0.0;
  }
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
  static const int _maxMissedFrames = 7;
  static const double _lockIouThreshold = 0.25;
  static const double _lockCenterThreshold = 0.18;
  static const double _initialProductPxM = 620.0;

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
  ObjectDetection? _lockedDetection;
  int _missedFrames = 0;
  double _distanceEwma = 2.5;
  bool _hasDistanceEwma = false;
  double _adaptiveProductPxM = _initialProductPxM;
  int _adaptiveSamples = 0;
  double? _prevDetectionWidth;

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
        _missedFrames++;
        if (_missedFrames >= _maxMissedFrames) {
          _lockedDetection = null;
          _prevDetectionWidth = null;
          _hasDistanceEwma = false;
        }
        setState(() {
          _distanceM = null;
          _collisionStatus = 'No target';
          _detections = [];
        });
        _updateSafetyState(SafetyState.unknown);
        return;
      }

      _missedFrames = 0;

      final locked = _pickLockedDetection(detections, image);
      if (locked == null) {
        setState(() {
          _distanceM = null;
          _collisionStatus = 'No stable target';
          _detections = detections;
        });
        _updateSafetyState(SafetyState.unknown);
        return;
      }

      _lockedDetection = locked;

      final estimatedDistance = _estimateDistanceFromLocked(locked, image.width);

      setState(() {
        _distanceM = estimatedDistance;
        _collisionStatus = 'Tracking stable target';
        _detections = [locked];
      });

      if (estimatedDistance < _dangerThresholdM) {
        _updateSafetyState(SafetyState.danger);
      } else if (estimatedDistance < _warningThresholdM) {
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

    // Grid motion map to separate independent moving regions.
    const int cellSize = 12;
    final cols = width ~/ cellSize;
    final rows = height ~/ cellSize;
    final active = List<bool>.filled(cols * rows, false);

    const int motionThreshold = 14;

    for (int y = 0; y < height; y += 6) {
      for (int x = 0; x < width; x += 6) {
        final index = y * yBytesPerRow + x;
        if (index < yPlane.length && index < previous.length) {
          final diff = (yPlane[index] - previous[index]).abs();
          if (diff > motionThreshold) {
            final cx = x ~/ cellSize;
            final cy = y ~/ cellSize;
            if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
              active[cy * cols + cx] = true;
            }
          }
        }
      }
    }

    _previousLuma = Uint8List.fromList(yPlane);

    return _clusterObjects(active, cols, rows, minPixels: 4, cellSize: cellSize);
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

    const int cellSize = 10;
    final cols = width ~/ cellSize;
    final rows = height ~/ cellSize;
    final active = List<bool>.filled(cols * rows, false);

    final targetR = _targetColor.red;
    final targetG = _targetColor.green;
    final targetB = _targetColor.blue;
    final maxDistanceSquared = _colorTolerance * _colorTolerance;

    for (int y = 0; y < height; y += 6) {
      for (int x = 0; x < width; x += 6) {
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
          final cx = x ~/ cellSize;
          final cy = y ~/ cellSize;
          if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            active[cy * cols + cx] = true;
          }
        }
      }
    }

    return _clusterObjects(active, cols, rows, minPixels: 3, cellSize: cellSize);
  }

  List<ObjectDetection> _clusterObjects(
    List<bool> pixelMap,
    int width,
    int height, {
    required int minPixels,
    required int cellSize,
  }) {
    final visited = List<bool>.filled(pixelMap.length, false);
    final out = <ObjectDetection>[];

    for (int idx = 0; idx < pixelMap.length; idx++) {
      if (!pixelMap[idx] || visited[idx]) continue;

      final queue = <int>[idx];
      visited[idx] = true;
      int count = 0;
      int minGX = idx % width;
      int maxGX = minGX;
      int minGY = idx ~/ width;
      int maxGY = minGY;

      while (queue.isNotEmpty && count < 500) {
        final cur = queue.removeLast();
        count++;
        final x = cur % width;
        final y = cur ~/ width;
        if (x < minGX) minGX = x;
        if (x > maxGX) maxGX = x;
        if (y < minGY) minGY = y;
        if (y > maxGY) maxGY = y;

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            final nx = x + dx;
            final ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            final ni = ny * width + nx;
            if (pixelMap[ni] && !visited[ni]) {
              visited[ni] = true;
              queue.add(ni);
            }
          }
        }
      }

      if (count < minPixels) continue;

      final minX = (minGX * cellSize).toDouble();
      final maxX = ((maxGX + 1) * cellSize).toDouble();
      final minY = (minGY * cellSize).toDouble();
      final maxY = ((maxGY + 1) * cellSize).toDouble();
      final frameMaxX = (width * cellSize).toDouble();
      final frameMaxY = (height * cellSize).toDouble();

      final boxW = maxX - minX;
      final boxH = maxY - minY;
      if (boxW < 24 || boxH < 24) continue;
      if (boxW * boxH < 1000) continue;

      // Expand slightly to capture full object contour instead of fragments.
      final padX = math.max(8.0, boxW * 0.10);
      final padY = math.max(8.0, boxH * 0.10);
      out.add(ObjectDetection(
        minX: (minX - padX).clamp(0.0, frameMaxX),
        maxX: (maxX + padX).clamp(0.0, frameMaxX),
        minY: (minY - padY).clamp(0.0, frameMaxY),
        maxY: (maxY + padY).clamp(0.0, frameMaxY),
      ));
    }

    out.sort((a, b) => b.area.compareTo(a.area));
    return out;
  }

  ObjectDetection? _pickLockedDetection(List<ObjectDetection> detections, CameraImage image) {
    if (detections.isEmpty) return null;
    if (_lockedDetection == null) {
      // Nearest proxy without hard calibration: largest width candidate.
      return detections.reduce((a, b) => a.width >= b.width ? a : b);
    }

    ObjectDetection? best;
    double bestScore = -1.0;
    for (final d in detections) {
      final iou = _lockedDetection!.iou(d);
      final dx = (d.center.dx - _lockedDetection!.center.dx) / image.width;
      final dy = (d.center.dy - _lockedDetection!.center.dy) / image.width;
      final centerScore = 1.0 - (dx * dx + dy * dy).clamp(0.0, 1.0);
      final score = (iou * 0.7) + (centerScore * 0.3);
      if (score > bestScore) {
        best = d;
        bestScore = score;
      }
    }

    if (best == null) return null;
    final iouOk = _lockedDetection!.iou(best) >= _lockIouThreshold;
    final dx = (best.center.dx - _lockedDetection!.center.dx) / image.width;
    final dy = (best.center.dy - _lockedDetection!.center.dy) / image.width;
    final centerOk = (dx * dx + dy * dy) <= (_lockCenterThreshold * _lockCenterThreshold);

    if (!iouOk && !centerOk) {
      return detections.reduce((a, b) => a.width >= b.width ? a : b);
    }
    return best;
  }

  double _estimateDistanceFromLocked(ObjectDetection detection, int imageWidth) {
    final widthPx = detection.width.clamp(12.0, imageWidth.toDouble());
    double rawDistance = (_adaptiveProductPxM / widthPx).clamp(0.18, 12.0);

    // CRITICAL: Enforce direction consistency so distance always follows width properly.
    // If object gets wider (closer), distance MUST decrease. If narrower (farther), distance MUST increase.
    final prevWidth = _prevDetectionWidth;
    final prevDist = _hasDistanceEwma ? _distanceEwma : null;

    if (prevWidth != null && prevDist != null && prevWidth > 10.0) {
      final widthRatio = widthPx / prevWidth;
      // If width increased (object closer), ensure distance decreased
      if (widthRatio > 1.02) {
        if (rawDistance > prevDist) {
          rawDistance = prevDist * 0.96;
        }
      }
      // If width decreased (object farther), ensure distance increased
      if (widthRatio < 0.98) {
        if (rawDistance < prevDist) {
          rawDistance = prevDist * 1.04;
        }
      }
    }

    // Keep the model adaptive, but much more conservative than the previous
    // correction-heavy version so the estimate follows the apparent size more
    // directly when you move closer or farther away.
    if (_hasDistanceEwma) {
      final observedProduct = widthPx * rawDistance;
      final boundedObs = observedProduct
          .clamp(_adaptiveProductPxM * 0.85, _adaptiveProductPxM * 1.15)
          .toDouble();
      _adaptiveProductPxM = _adaptiveProductPxM * 0.97 + boundedObs * 0.03;
    }
    _adaptiveSamples = math.min(100000, _adaptiveSamples + 1);

    // Smooth output while preserving responsive behavior.
    if (!_hasDistanceEwma) {
      _distanceEwma = rawDistance;
      _hasDistanceEwma = true;
    } else {
      // Shorter distances need faster response, so tighten smoothing for close range.
      final alpha = rawDistance < 1.2 ? 0.25 : 0.15;
      _distanceEwma = (_distanceEwma * (1.0 - alpha)) + (rawDistance * alpha);
    }

    _prevDetectionWidth = widthPx;
    return _distanceEwma.clamp(0.18, 12.0);
  }

  void _resetTrackingState() {
    _distanceM = null;
    _collisionStatus = 'Scanning';
    _previousLuma = null;
    _detections = [];
    _lockedDetection = null;
    _missedFrames = 0;
    _prevDetectionWidth = null;
    _hasDistanceEwma = false;
    _distanceEwma = 2.5;
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
                          _resetTrackingState();
                          _collisionStatus = 'Mode switched';
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
