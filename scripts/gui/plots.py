"""
Dynamic plot widgets using pyqtgraph.
Supports S-parameters, spectrum, and gain plots.
"""
from __future__ import annotations

from typing import Dict, List, Optional
import numpy as np
import pyqtgraph as pg

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets

from gui.styles import COLORS


class DynamicPlotWidget(QtWidgets.QWidget):
    """Dynamic plot that changes based on measurement type."""
    
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._sparam_series: List[Dict[str, object]] = []
        self._spectrum_data: Dict[str, object] = {}  # Store spectrum plot data
        self._mag_mouse_proxy = None
        self._phase_mouse_proxy = None
        self._single_mouse_proxy = None
        self._setup_ui()
        self._current_mode = "s_parameter"
    
    def _setup_ui(self):
        """Setup the plot UI."""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create plot widget with dark background
        pg.setConfigOption('background', COLORS['background'])
        pg.setConfigOption('foreground', COLORS['text'])
        
        # Add title label for single plot
        self.single_plot_title = QtWidgets.QLabel("")
        self.single_plot_title.setStyleSheet(f"font-weight: bold; font-size: 14px; color: {COLORS['text']}; padding: 4px;")
        self.single_plot_title.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.single_plot_title)
        
        self.single_plot_widget = pg.PlotWidget()
        self.single_plot_widget.setBackground(COLORS['background'])

        self.sparam_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        mag_container = QtWidgets.QWidget()
        mag_layout = QtWidgets.QVBoxLayout(mag_container)
        mag_layout.setContentsMargins(0, 0, 0, 0)
        mag_layout.setSpacing(4)
        self.mag_title = QtWidgets.QLabel("Magnitude")
        self.mag_title.setStyleSheet(f"font-weight: bold; color: {COLORS['text']};")
        self.mag_plot_widget = pg.PlotWidget()
        mag_layout.addWidget(self.mag_title)
        mag_layout.addWidget(self.mag_plot_widget)

        phase_container = QtWidgets.QWidget()
        phase_layout = QtWidgets.QVBoxLayout(phase_container)
        phase_layout.setContentsMargins(0, 0, 0, 0)
        phase_layout.setSpacing(4)
        self.phase_title = QtWidgets.QLabel("Phase")
        self.phase_title.setStyleSheet(f"font-weight: bold; color: {COLORS['text']};")
        self.phase_plot_widget = pg.PlotWidget()
        phase_layout.addWidget(self.phase_title)
        phase_layout.addWidget(self.phase_plot_widget)

        self.mag_plot_widget.setBackground(COLORS['background'])
        self.phase_plot_widget.setBackground(COLORS['background'])
        self.sparam_splitter.addWidget(mag_container)
        self.sparam_splitter.addWidget(phase_container)
        self.sparam_splitter.setStretchFactor(0, 1)
        self.sparam_splitter.setStretchFactor(1, 1)

        layout.addWidget(self.single_plot_widget)
        layout.addWidget(self.sparam_splitter)
        
        # Configure axes
        self.single_plot_widget.setLabel('bottom', 'Frequency', units='GHz')
        self.single_plot_widget.setLabel('left', 'Magnitude', units='dB')
        self.single_plot_widget.showGrid(x=True, y=True, alpha=0.3)

        self.mag_plot_widget.setLabel('bottom', 'Frequency', units='Hz')
        self.mag_plot_widget.setLabel('left', 'Magnitude', units='dB')
        self.mag_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.mag_plot_widget.addLegend()

        self.phase_plot_widget.setLabel('bottom', 'Frequency', units='Hz')
        self.phase_plot_widget.setLabel('left', 'Phase', units='deg')
        self.phase_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.phase_plot_widget.addLegend()

        self.sparam_splitter.hide()

        # Comparison mode containers (for golden vs tested comparison)
        self.comparison_container = QtWidgets.QWidget()
        self.comparison_layout = QtWidgets.QVBoxLayout(self.comparison_container)
        self.comparison_layout.setContentsMargins(0, 0, 0, 0)

        # Top row: Golden plots
        self.golden_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.golden_gain_plot = self._create_comparison_plot("Golden Gain")
        self.golden_phase_plot = self._create_comparison_plot("Golden Phase")
        self.golden_mag_plot = self._create_comparison_plot("Golden S11 Magnitude")
        self.golden_row.addWidget(self.golden_gain_plot[0])
        self.golden_row.addWidget(self.golden_phase_plot[0])
        self.golden_row.addWidget(self.golden_mag_plot[0])

        # Bottom row: Tested plots
        self.tested_row = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.tested_gain_plot = self._create_comparison_plot("Tested Gain")
        self.tested_phase_plot = self._create_comparison_plot("Tested Phase")
        self.tested_mag_plot = self._create_comparison_plot("Tested S11 Magnitude")
        self.tested_row.addWidget(self.tested_gain_plot[0])
        self.tested_row.addWidget(self.tested_phase_plot[0])
        self.tested_row.addWidget(self.tested_mag_plot[0])

        self.comparison_layout.addWidget(self.golden_row)
        self.comparison_layout.addWidget(self.tested_row)
        self.comparison_container.hide()

        layout.addWidget(self.comparison_container)

        self._mag_mouse_proxy = pg.SignalProxy(
            self.mag_plot_widget.scene().sigMouseMoved,
            rateLimit=60,
            slot=self._on_mag_mouse_moved,
        )
        self._phase_mouse_proxy = pg.SignalProxy(
            self.phase_plot_widget.scene().sigMouseMoved,
            rateLimit=60,
            slot=self._on_phase_mouse_moved,
        )
        self.mag_plot_widget.scene().sigMouseClicked.connect(self._on_mag_mouse_clicked)
        self.phase_plot_widget.scene().sigMouseClicked.connect(self._on_phase_mouse_clicked)
        
        # Add mouse tracking for single plot (spectrum/gain)
        self._single_mouse_proxy = pg.SignalProxy(
            self.single_plot_widget.scene().sigMouseMoved,
            rateLimit=60,
            slot=self._on_single_mouse_moved,
        )
        self.single_plot_widget.scene().sigMouseClicked.connect(self._on_single_mouse_clicked)
    
    def set_mode(self, mode: str):
        """Set the plot mode (s_parameter, spectrum, gain, comparison)."""
        self._current_mode = mode
        self.clear()

        # Hide all containers first
        self.single_plot_widget.hide()
        self.single_plot_title.hide()
        self.sparam_splitter.hide()
        self.comparison_container.hide()

        if mode == "comparison":
            self.comparison_container.show()
            self.golden_row.show()
            self.tested_row.show()
        elif mode == "s_parameter":
            self.single_plot_widget.hide()
            self.single_plot_title.hide()
            self.sparam_splitter.show()
            self.mag_plot_widget.setLabel('bottom', 'Frequency', units='Hz')
            self.mag_plot_widget.setLabel('left', 'Magnitude', units='dB')
            self.phase_plot_widget.setLabel('bottom', 'Frequency', units='Hz')
            self.phase_plot_widget.setLabel('left', 'Phase', units='deg')
            self.mag_title.setText("S-Parameters: Magnitude")
            self.phase_title.setText("S-Parameters: Phase")
        elif mode == "spectrum":
            self.sparam_splitter.hide()
            self.single_plot_widget.show()
            self.single_plot_title.show()
            self.single_plot_title.setText("tinySA Spectrum Analysis")
            self.single_plot_widget.setLabel('left', 'Power', units='dBm')
            self.single_plot_widget.setLabel('bottom', 'Frequency', units='GHz')
        elif mode == "gain":
            self.sparam_splitter.hide()
            self.single_plot_widget.show()
            self.single_plot_title.show()
            self.single_plot_title.setText("Gain Measurement")
            self.single_plot_widget.setLabel('left', 'Gain', units='dB')
            self.single_plot_widget.setLabel('bottom', 'Frequency', units='GHz')
    
    def clear(self):
        """Clear the plot."""
        self.single_plot_widget.clear()
        self.mag_plot_widget.clear()
        self.phase_plot_widget.clear()
        # Clear comparison plots
        if hasattr(self, 'golden_gain_plot'):
            self.golden_gain_plot[1].clear()
            self.golden_phase_plot[1].clear()
            self.golden_mag_plot[1].clear()
            self.tested_gain_plot[1].clear()
            self.tested_phase_plot[1].clear()
            self.tested_mag_plot[1].clear()
        self._sparam_series = []
        self._spectrum_data = {}  # Clear spectrum data
    
    def plot_data(self, x_data, y_data, name="", color=None):
        """Plot data on the widget."""
        if color is None:
            color = COLORS['accent']
        pen = pg.mkPen(color=color, width=2)
        self.single_plot_widget.plot(x_data, y_data, pen=pen, name=name)
        self._auto_range_plot(self.single_plot_widget)
        
        # Store data for hover/click info
        self._spectrum_data = {
            "x": list(x_data),
            "y": list(y_data),
            "name": name,
        }

    @staticmethod
    def _open_curve_points(points: list) -> tuple:
        """Return (xs, ys) in original order, with duplicate last point removed to avoid closing the path."""
        if not points:
            return ([], [])
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        # If last point equals first (closed contour), drop last so no line from end back to start
        if len(xs) > 1 and xs[0] == xs[-1] and ys[0] == ys[-1]:
            xs = xs[:-1]
            ys = ys[:-1]
        return (xs, ys)

    def plot_sparameters(self, payload: dict):
        """Plot S-parameter magnitude and phase in two columns."""
        self.set_mode("s_parameter")
        self.clear()

        mag = payload.get("mag", {}) if isinstance(payload, dict) else {}
        phase = payload.get("phase", {}) if isinstance(payload, dict) else {}
        show_mag = set(payload.get("show_mag", [])) if isinstance(payload, dict) else set()
        show_phase = set(payload.get("show_phase", [])) if isinstance(payload, dict) else set()
        colors = payload.get("colors", {}) if isinstance(payload, dict) else {}
        self._sparam_series = []

        for param in sorted(set(list(mag.keys()) + list(phase.keys()))):
            color = colors.get(param, COLORS['accent'])
            pen = pg.mkPen(color=color, width=2)

            if param in show_mag and isinstance(mag.get(param), list) and mag.get(param):
                xs, ys = self._open_curve_points(mag[param])
                if xs:
                    self.mag_plot_widget.plot(
                        xs, ys, pen=pen, name=f"{param} (Mag)", connect="finite"
                    )
                    self._sparam_series.append({
                        "plot": "mag",
                        "param": param,
                        "unit": "dB",
                        "x": xs,
                        "y": ys,
                    })

            if param in show_phase and isinstance(phase.get(param), list) and phase.get(param):
                xs, ys = self._open_curve_points(phase[param])
                if xs:
                    self.phase_plot_widget.plot(
                        xs, ys, pen=pen, name=f"{param} (Phase)", connect="finite"
                    )
                    self._sparam_series.append({
                        "plot": "phase",
                        "param": param,
                        "unit": "deg",
                        "x": xs,
                        "y": ys,
                    })

        self._auto_range_plot(self.mag_plot_widget)
        self._auto_range_plot(self.phase_plot_widget)

    def _create_comparison_plot(self, title: str):
        """Create a labeled plot widget for comparison view."""
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        label = QtWidgets.QLabel(title)
        label.setStyleSheet(f"font-weight: bold; color: {COLORS['text']}; font-size: 11px;")
        label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(label)

        plot_widget = pg.PlotWidget()
        plot_widget.setBackground(COLORS['background'])
        plot_widget.setLabel('bottom', 'Frequency', units='GHz')
        plot_widget.setLabel('left', 'Value')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(plot_widget)

        return (container, plot_widget)

    def plot_comparison(self, golden_record: Optional[dict], tested_record: Optional[dict],
                        is_single_record: bool = False,
                        meas_type: str = "both"):
        """Plot comparison between golden and tested antenna records.

        Args:
            golden_record: Golden sample record (can be None in single-record mode)
            tested_record: Tested antenna record (can be None in single-record mode)
            is_single_record: If True, only one record is selected (not a comparison).
                              The other row will be hidden.
            meas_type: Measurement type - "gain_only", "sparam_only", or "both".
                       Determines the layout of plots:
                       - gain_only: 2 columns, gain plots (for tinySA-only records)
                       - sparam_only: 2 columns, phase | magnitude (for LibreVNA-only records)
                       - both: 2x3 grid, gain | phase | magnitude (for mixed records)

        Layouts per type:
        - gain_only: 2 columns (golden gain | tested gain) - or single column for single record
        - sparam_only: 2 columns (golden phase/mag | tested phase/mag)
        - both: 2x3 grid (golden gain/phase/mag | tested gain/phase/mag)
        - Single record (any type): only the relevant row is shown
        """
        self.set_mode("comparison")
        self.clear()

        # Show/hide individual plot containers based on meas_type
        if meas_type == "gain_only":
            self.golden_gain_plot[0].show()
            self.golden_phase_plot[0].hide()
            self.golden_mag_plot[0].hide()
            # For gain_only mode in comparison (both golden and tested selected),
            # we need both gain plots visible side by side
            if not is_single_record:
                self.tested_gain_plot[0].show()
            else:
                self.tested_gain_plot[0].hide()
            self.tested_phase_plot[0].hide()
            self.tested_mag_plot[0].hide()
        elif meas_type == "sparam_only":
            self.golden_gain_plot[0].hide()
            self.golden_phase_plot[0].show()
            self.golden_mag_plot[0].show()
            self.tested_gain_plot[0].hide()
            self.tested_phase_plot[0].show()
            self.tested_mag_plot[0].show()
        else:  # "both"
            self.golden_gain_plot[0].show()
            self.golden_phase_plot[0].show()
            self.golden_mag_plot[0].show()
            self.tested_gain_plot[0].show()
            self.tested_phase_plot[0].show()
            self.tested_mag_plot[0].show()

        # Row visibility for single-record mode
        if is_single_record:
            if not golden_record:
                self.golden_row.hide()
            else:
                self.golden_row.show()
            if not tested_record:
                self.tested_row.hide()
            else:
                self.tested_row.show()
        else:
            self.golden_row.show()
            self.tested_row.show()

        # --- Plot golden data ---
        if golden_record:
            self._plot_record_into_comparison_containers(
                golden_record, self.golden_gain_plot[1], self.golden_phase_plot[1],
                self.golden_mag_plot[1], meas_type, "golden"
            )

        # --- Plot tested data ---
        if tested_record:
            self._plot_record_into_comparison_containers(
                tested_record, self.tested_gain_plot[1], self.tested_phase_plot[1],
                self.tested_mag_plot[1], meas_type, "tested"
            )

    def _plot_record_into_comparison_containers(self, record: dict,
                                                gain_plot: pg.PlotWidget,
                                                phase_plot: pg.PlotWidget,
                                                mag_plot: pg.PlotWidget,
                                                meas_type: str, source: str):
        """Plot a record's data into the appropriate comparison containers.

        Args:
            record: Antenna record dict
            gain_plot: PlotWidget for gain
            phase_plot: PlotWidget for phase
            mag_plot: PlotWidget for S11 magnitude
            meas_type: "gain_only", "sparam_only", or "both"
            source: "golden" or "tested" (used for naming and color)
        """
        color = COLORS['accent'] if source == "golden" else COLORS['accent2']
        pen = pg.mkPen(color=color, width=2)

        # --- Gain data ---
        if meas_type in ("gain_only", "both"):
            # Try multiple sources for gain data
            measurements = record.get("measurements", [])
            gain_data = record.get("gain_data", {})
            points = gain_data.get("points", []) if isinstance(gain_data, dict) else []

            x_data = []
            y_data = []
            for point in (measurements or points):
                freq = point.get("frequency_hz", 0)
                value = point.get("power_dbm") or point.get("gain_db") or point.get("power", 0)
                if value is None:
                    continue
                x_data.append(freq / 1e9)
                y_data.append(float(value))

            if x_data:
                x_arr = np.array(x_data, dtype=float)
                y_arr = np.array(y_data, dtype=float)
                
                n = len(x_arr)
                if n >= 2:
                    # Create explicit segment connections: prevent first-to-last connection
                    connect = np.ones(n, dtype=np.int8)
                    connect[0] = 1   # first: connect to next only
                    connect[-1] = 4  # last: connect to previous only
                    if n > 2:
                        connect[1:-1] = 2  # middle: connect to both
                    gain_plot.plot(x_arr, y_arr, pen=pen, name=f"{source.capitalize()} Gain", connect=connect)
                else:
                    # Single point - no connection needed
                    gain_plot.plot(x_arr, y_arr, pen=pen, name=f"{source.capitalize()} Gain", connect=np.array([0], dtype=np.int8))
                gain_plot.setLabel('left', 'Gain', units='dB')
                gain_plot.setLabel('bottom', 'Frequency', units='GHz')
                self._auto_range_plot(gain_plot)
            else:
                # Hide the gain container if no gain data
                gain_plot.getViewBox().parent().parent().hide()

        # --- S11 data (phase + magnitude) ---
        if meas_type in ("sparam_only", "both"):
            s11_data = record.get("s11_data", {})
            frequencies = s11_data.get("frequencies", []) if isinstance(s11_data, dict) else []
            magnitudes = s11_data.get("magnitudes", []) if isinstance(s11_data, dict) else []
            phases = s11_data.get("phases", []) if isinstance(s11_data, dict) else []

            if frequencies and magnitudes:
                x_arr = np.array([f / 1e9 for f in frequencies], dtype=float)
                mag_arr = np.array(magnitudes, dtype=float)

                # Debug logging
                import sys
                print(f"[PLOT DEBUG] {source} - freqs: {len(x_arr)}, first={x_arr[0]:.6f}, last={x_arr[-1]:.6f}", file=sys.stderr)
                print(f"[PLOT DEBUG] {source} - mags: {len(mag_arr)}, first={mag_arr[0]:.2f}, last={mag_arr[-1]:.2f}", file=sys.stderr)

                n = len(x_arr)
                if n >= 2:
                    # Create explicit segment connections: [1, 2, 2, ..., 2, 4]
                    # 1 = connect to next only, 2 = connect to both, 4 = connect to previous only
                    connect = np.ones(n, dtype=np.int8)
                    connect[0] = 1   # first: connect to next only
                    connect[-1] = 4  # last: connect to previous only
                    if n > 2:
                        connect[1:-1] = 2  # middle: connect to both
                    
                    # Magnitude
                    mag_plot.plot(x_arr, mag_arr, pen=pen, name=f"{source.capitalize()} S11", connect=connect)
                    mag_plot.setLabel('left', 'S11', units='dB')
                    mag_plot.setLabel('bottom', 'Frequency', units='GHz')
                    self._auto_range_plot(mag_plot)
                    print(f"[PLOT DEBUG] {source} - plotted magnitude with explicit connect segments", file=sys.stderr)

                    # Phase
                    if phases and len(phases) == n:
                        phase_arr = np.array(phases, dtype=float)
                        phase_plot.plot(x_arr, phase_arr, pen=pen, name=f"{source.capitalize()} Phase", connect=connect)
                        phase_plot.setLabel('left', 'Phase', units='deg')
                        phase_plot.setLabel('bottom', 'Frequency', units='GHz')
                        self._auto_range_plot(phase_plot)
                        print(f"[PLOT DEBUG] {source} - plotted phase with explicit connect segments", file=sys.stderr)
                    else:
                        phase_plot.getViewBox().parent().parent().hide()
                else:
                    # Not enough points to plot
                    print(f"[PLOT DEBUG] {source} - not enough points ({n}) to plot", file=sys.stderr)
                    mag_plot.getViewBox().parent().parent().hide()
                    phase_plot.getViewBox().parent().parent().hide()
            else:
                # Hide both S11 containers if no S11 data
                phase_plot.getViewBox().parent().parent().hide()
                mag_plot.getViewBox().parent().parent().hide()

    def _on_mag_mouse_moved(self, evt):
        self._show_hover_info(evt, "mag")

    def _on_phase_mouse_moved(self, evt):
        self._show_hover_info(evt, "phase")

    def _on_mag_mouse_clicked(self, event):
        self._show_click_info(event, "mag")

    def _on_phase_mouse_clicked(self, event):
        self._show_click_info(event, "phase")

    def _on_single_mouse_moved(self, evt):
        """Handle mouse movement on single plot (spectrum/gain)."""
        if self._current_mode not in ("spectrum", "gain"):
            return
        pos = evt[0] if isinstance(evt, (list, tuple)) and evt else evt
        info = self._nearest_single_point(pos)
        if not info:
            return
        
        # Format based on mode
        if self._current_mode == "spectrum":
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                f"f={info['freq']:.6f} GHz | Power={info['value']:.2f} dBm",
                self,
            )
        elif self._current_mode == "gain":
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                f"f={info['freq']:.6f} GHz | Gain={info['value']:.2f} dB",
                self,
            )

    def _on_single_mouse_clicked(self, event):
        """Handle mouse click on single plot (spectrum/gain)."""
        if self._current_mode not in ("spectrum", "gain") or event is None:
            return
        info = self._nearest_single_point(event.scenePos())
        if not info:
            return
        
        # Format based on mode
        if self._current_mode == "spectrum":
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                f"Selected Point #{info['index']} | f={info['freq']:.6f} GHz | Power={info['value']:.2f} dBm",
                self,
            )
        elif self._current_mode == "gain":
            QtWidgets.QToolTip.showText(
                QtGui.QCursor.pos(),
                f"Selected Point #{info['index']} | f={info['freq']:.6f} GHz | Gain={info['value']:.2f} dB",
                self,
            )

    def _show_hover_info(self, evt, plot_kind: str):
        if self._current_mode != "s_parameter":
            return
        pos = evt[0] if isinstance(evt, (list, tuple)) and evt else evt
        info = self._nearest_series_point(pos, plot_kind)
        if not info:
            return
        QtWidgets.QToolTip.showText(
            QtGui.QCursor.pos(),
            f"{info['param']} | f={info['freq_hz']:.3f} Hz | {info['value']:.2f} {info['unit']}",
            self,
        )

    def _show_click_info(self, event, plot_kind: str):
        if self._current_mode != "s_parameter" or event is None:
            return
        info = self._nearest_series_point(event.scenePos(), plot_kind)
        if not info:
            return
        QtWidgets.QToolTip.showText(
            QtGui.QCursor.pos(),
            f"Selected {info['param']} | Point #{info['index']} | f={info['freq_hz']:.3f} Hz | {info['value']:.2f} {info['unit']}",
            self,
        )

    def _nearest_series_point(self, scene_pos, plot_kind: str):
        plot = self.mag_plot_widget if plot_kind == "mag" else self.phase_plot_widget
        if scene_pos is None or not plot.sceneBoundingRect().contains(scene_pos):
            return None

        vb = plot.getPlotItem().vb
        mouse_point = vb.mapSceneToView(scene_pos)
        mx = float(mouse_point.x())
        my = float(mouse_point.y())

        nearest = None
        nearest_d2 = None
        for series in self._sparam_series:
            if series.get("plot") != plot_kind:
                continue
            xs = series.get("x", [])
            ys = series.get("y", [])
            for idx, (x, y) in enumerate(zip(xs, ys)):
                view_pt = QtCore.QPointF(float(x), float(y))
                scene_pt = vb.mapViewToScene(view_pt)
                px = float(scene_pt.x()) - float(scene_pos.x())
                py = float(scene_pt.y()) - float(scene_pos.y())
                d2 = px * px + py * py
                if nearest_d2 is None or d2 < nearest_d2:
                    nearest_d2 = d2
                    nearest = {
                        "param": str(series.get("param", "")),
                        "index": idx,
                        "freq_hz": float(x),
                        "value": float(y),
                        "unit": str(series.get("unit", "")),
                    }
        if nearest is None:
            return None
        if nearest_d2 is not None and nearest_d2 > (30.0 * 30.0):
            return None
        return nearest

    def _nearest_single_point(self, scene_pos):
        """Find nearest point in single plot (spectrum/gain)."""
        if not self._spectrum_data or scene_pos is None:
            return None
        
        plot = self.single_plot_widget
        if not plot.sceneBoundingRect().contains(scene_pos):
            return None
        
        vb = plot.getPlotItem().vb
        mouse_point = vb.mapSceneToView(scene_pos)
        mx = float(mouse_point.x())
        my = float(mouse_point.y())
        
        xs = self._spectrum_data.get("x", [])
        ys = self._spectrum_data.get("y", [])
        
        if not xs or not ys:
            return None
        
        nearest = None
        nearest_d2 = None
        for idx, (x, y) in enumerate(zip(xs, ys)):
            view_pt = QtCore.QPointF(float(x), float(y))
            scene_pt = vb.mapViewToScene(view_pt)
            px = float(scene_pt.x()) - float(scene_pos.x())
            py = float(scene_pt.y()) - float(scene_pos.y())
            d2 = px * px + py * py
            if nearest_d2 is None or d2 < nearest_d2:
                nearest_d2 = d2
                nearest = {
                    "index": idx,
                    "freq": float(x),
                    "value": float(y),
                }
        
        if nearest is None:
            return None
        if nearest_d2 is not None and nearest_d2 > (30.0 * 30.0):
            return None
        return nearest

    @staticmethod
    def _auto_range_plot(plot_widget: pg.PlotWidget):
        """Auto-fit visible data with a small padding."""
        item = plot_widget.getPlotItem()
        if item is None or not item.listDataItems():
            return
        plot_widget.enableAutoRange(axis='xy', enable=True)
        plot_widget.autoRange(padding=0.05)
