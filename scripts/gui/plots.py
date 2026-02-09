"""
Dynamic plot widgets using pyqtgraph.
Supports S-parameters, spectrum, and gain plots.
"""
from __future__ import annotations

from typing import Dict, List, Optional
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
        """Set the plot mode (s_parameter, spectrum, gain)."""
        self._current_mode = mode
        self.clear()
        
        if mode == "s_parameter":
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
                xs = [x for x, _ in mag[param]]
                ys = [y for _, y in mag[param]]
                self.mag_plot_widget.plot(xs, ys, pen=pen, name=f"{param} (Mag)")
                self._sparam_series.append({
                    "plot": "mag",
                    "param": param,
                    "unit": "dB",
                    "x": xs,
                    "y": ys,
                })

            if param in show_phase and isinstance(phase.get(param), list) and phase.get(param):
                xs = [x for x, _ in phase[param]]
                ys = [y for _, y in phase[param]]
                self.phase_plot_widget.plot(xs, ys, pen=pen, name=f"{param} (Phase)")
                self._sparam_series.append({
                    "plot": "phase",
                    "param": param,
                    "unit": "deg",
                    "x": xs,
                    "y": ys,
                })

        self._auto_range_plot(self.mag_plot_widget)
        self._auto_range_plot(self.phase_plot_widget)

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
