// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QHash>
#include <QtCore/QPointF>
#include <QtCore/QVector>
#include <QtGui/QColor>
#include <QtCore/QElapsedTimer>

class QGestureEvent;
class QPinchGesture;
class QTapGesture;
class QPanGesture;

namespace ui::widgets {

class PlotView : public QChartView {
    Q_OBJECT

public:
    explicit PlotView(QWidget* parent = nullptr);

    void setSeriesColor(const QString& name, const QColor& color);
    void setSeriesMagnitudeColor(const QString& name, const QColor& color);
    void setSeriesPhaseColor(const QString& name, const QColor& color);
    void setSeriesVisible(const QString& name, bool on);
    void setSeriesMagnitudeVisible(const QString& name, bool on);
    void setSeriesPhaseVisible(const QString& name, bool on);
    void setShowMagnitude(bool on);
    void setShowPhase(bool on);
    void setSeriesData(const QString& name,
                       const QVector<QPointF>& magnitudePoints,
                       const QVector<QPointF>& phasePoints);
    void setFrequencyAxisRange(double minGHz, double maxGHz);
    void setMagnitudeAxisRange(double minDb, double maxDb);
    void setPhaseAxisRange(double minDeg, double maxDeg);
    void applyTranslations(const QHash<QString, QString>& strings);

signals:
    void rangeChanged(double startGHz, double endGHz);

protected:
    bool event(QEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    struct SeriesSet {
        QLineSeries* magnitude = nullptr;
        QLineSeries* phase = nullptr;
        QColor magnitudeColor;
        QColor phaseColor;
        bool enabled = true;
        bool magnitudeVisible = true;
        bool phaseVisible = true;
    };

    void initializeChart();
    void initializeSeries();
    void updateSeriesVisibility(const QString& name);
    void updateGlobalVisibility();
    void updateAxisTitles();
    void refreshSeriesPens(const QString& name);
    void clampFrequencyRange();
    bool handleGesture(QGestureEvent* event);
    void handlePinchGesture(QPinchGesture* pinch);
    void handlePanGesture(QPanGesture* pan);
    void handleTapGesture(QTapGesture* tap);
    void resetZoom();
    void applyAxisZoom(QValueAxis* axis,
                       double startMin,
                       double startMax,
                       double scaleFactor,
                       double minSpan,
                       double maxSpan,
                       bool enforceNonNegative = false);

    QHash<QString, SeriesSet> m_series;
    QHash<QString, QString> m_strings;
    QValueAxis* m_axisFrequency = nullptr;
    QValueAxis* m_axisMagnitude = nullptr;
    QValueAxis* m_axisPhase = nullptr;
    bool m_showMagnitude = true;
    bool m_showPhase = true;
    bool m_pinchActive = false;
    bool m_panActive = false;
    double m_pinchStartFreqMin = 0.0;
    double m_pinchStartFreqMax = 0.0;
    double m_pinchStartMagMin = 0.0;
    double m_pinchStartMagMax = 0.0;
    double m_pinchStartPhaseMin = 0.0;
    double m_pinchStartPhaseMax = 0.0;
    double m_panStartFreqMin = 0.0;
    double m_panStartFreqMax = 0.0;
    double m_panStartMagMin = 0.0;
    double m_panStartMagMax = 0.0;
    double m_panStartPhaseMin = 0.0;
    double m_panStartPhaseMax = 0.0;
    QElapsedTimer m_tapTimer;
    int m_tapCount = 0;
    const double m_defaultFreqMin = 1.0;
    const double m_defaultFreqMax = 6.0;
    const double m_defaultMagMin = -80.0;
    const double m_defaultMagMax = 10.0;
    const double m_defaultPhaseMin = -180.0;
    const double m_defaultPhaseMax = 180.0;
};

} // namespace ui::widgets

