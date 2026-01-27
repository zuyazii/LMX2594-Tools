#include "PlotView.hpp"

#include "../theme/DesignTokens.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QValueAxis>
#include <QtCore/QMargins>
#include <QtCore/QPointF>
#include <QtCore/QtMath>
#include <QGestureEvent>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGui/QResizeEvent>
#include <QtWidgets/QGesture>
#include <QtWidgets/QPanGesture>
#include <QtWidgets/QPinchGesture>
#include <QtWidgets/QTapGesture>
#include <algorithm>
#include <cmath>

namespace ui::widgets {

using ui::theme::Tokens::BG_Canvas;
using ui::theme::Tokens::InkMuted;
using ui::theme::Tokens::InkPrimary;
using ui::theme::Tokens::S11;
using ui::theme::Tokens::S12;
using ui::theme::Tokens::S21;
using ui::theme::Tokens::S22;
using ui::theme::Tokens::StrokeGrid;
using ui::theme::Tokens::StrokeSoft;

namespace {

QPen makePen(const QColor& color, Qt::PenStyle style = Qt::SolidLine) {
    QPen pen(color, 1.6);
    pen.setStyle(style);
    pen.setCosmetic(true);
    return pen;
}

} // namespace

PlotView::PlotView(QWidget* parent)
    : QChartView(parent) {
    grabGesture(Qt::PinchGesture);
    grabGesture(Qt::PanGesture);
    grabGesture(Qt::TapGesture);
    initializeChart();
    initializeSeries();
    updateAxisTitles();
    updateGlobalVisibility();
}

void PlotView::setSeriesColor(const QString& name, const QColor& color) {
    setSeriesMagnitudeColor(name, color);
    setSeriesPhaseColor(name, color);
}

void PlotView::setSeriesMagnitudeColor(const QString& name, const QColor& color) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    it->magnitudeColor = color;
    refreshSeriesPens(name);
}

void PlotView::setSeriesPhaseColor(const QString& name, const QColor& color) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    it->phaseColor = color;
    refreshSeriesPens(name);
}

void PlotView::setSeriesVisible(const QString& name, bool on) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    it->enabled = on;
    updateSeriesVisibility(name);
}

void PlotView::setSeriesMagnitudeVisible(const QString& name, bool on) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    it->magnitudeVisible = on;
    updateSeriesVisibility(name);
}

void PlotView::setSeriesPhaseVisible(const QString& name, bool on) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    it->phaseVisible = on;
    updateSeriesVisibility(name);
}

void PlotView::setShowMagnitude(bool on) {
    if (m_showMagnitude == on) {
        return;
    }
    m_showMagnitude = on;
    updateGlobalVisibility();
}

void PlotView::setShowPhase(bool on) {
    if (m_showPhase == on) {
        return;
    }
    m_showPhase = on;
    updateGlobalVisibility();
}

void PlotView::setSeriesData(const QString& name,
                       const QVector<QPointF>& magnitudePoints,
                       const QVector<QPointF>& phasePoints) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    if (it->magnitude) {
        it->magnitude->replace(magnitudePoints);
    }
    if (it->phase) {
        it->phase->replace(phasePoints);
    }
}

bool PlotView::event(QEvent* event) {
    if (event->type() == QEvent::Gesture &&
        handleGesture(static_cast<QGestureEvent*>(event))) {
        return true;
    }
    return QChartView::event(event);
}

void PlotView::resizeEvent(QResizeEvent* event) {
    QChartView::resizeEvent(event);
    clampFrequencyRange();
}

void PlotView::applyTranslations(const QHash<QString, QString>& strings) {
    m_strings = strings;
    updateAxisTitles();
}

void PlotView::initializeChart() {
    setObjectName(QStringLiteral("PlotView"));
    setRenderHint(QPainter::Antialiasing, true);
    setRubberBand(QChartView::RectangleRubberBand);
    setContentsMargins(0, 0, 0, 0);
    setBackgroundBrush(Qt::NoBrush);
    setAttribute(Qt::WA_StyledBackground, true);

    QChart* c = chart();
    c->setBackgroundBrush(QColor(QString::fromUtf8(BG_Canvas)));
    c->setBackgroundPen(Qt::NoPen);
    c->legend()->hide();
    c->setMargins(QMargins(16, 16, 16, 16));

    m_axisFrequency = new QValueAxis;
    m_axisFrequency->setTitleText(QStringLiteral("Frequency (GHz)"));
    m_axisFrequency->setRange(m_defaultFreqMin, m_defaultFreqMax);
    m_axisFrequency->setMinorTickCount(1);

    m_axisMagnitude = new QValueAxis;
    m_axisMagnitude->setTitleText(QStringLiteral("Magnitude (dB)"));
    m_axisMagnitude->setRange(m_defaultMagMin, m_defaultMagMax);
    m_axisMagnitude->setMinorTickCount(1);

    m_axisPhase = new QValueAxis;
    m_axisPhase->setTitleText(QStringLiteral("Phase (deg)"));
    m_axisPhase->setRange(m_defaultPhaseMin, m_defaultPhaseMax);
    m_axisPhase->setMinorTickCount(1);

    const QPen gridPen(QColor(QString::fromUtf8(StrokeGrid)));
    const QPen axisPen(QColor(QString::fromUtf8(StrokeSoft)));

    for (QValueAxis* axis : {m_axisFrequency, m_axisMagnitude, m_axisPhase}) {
        axis->setGridLinePen(gridPen);
        axis->setMinorGridLinePen(gridPen);
        axis->setLinePen(axisPen);
        axis->setLabelsColor(QColor(QString::fromUtf8(InkPrimary)));
        axis->setTitleBrush(QBrush(QColor(QString::fromUtf8(InkPrimary))));
    }

    QFont axisFont;
    axisFont.setPointSize(ui::theme::Tokens::Font::SM);
    axisFont.setFamily(ui::theme::Tokens::fontFamily());

    m_axisFrequency->setLabelsFont(axisFont);
    m_axisFrequency->setTitleFont(axisFont);
    m_axisMagnitude->setLabelsFont(axisFont);
    m_axisMagnitude->setTitleFont(axisFont);
    m_axisPhase->setLabelsFont(axisFont);
    m_axisPhase->setTitleFont(axisFont);

    c->addAxis(m_axisFrequency, Qt::AlignBottom);
    c->addAxis(m_axisMagnitude, Qt::AlignLeft);
    c->addAxis(m_axisPhase, Qt::AlignRight);

    connect(m_axisFrequency, &QValueAxis::rangeChanged, this, [this](qreal min, qreal max) {
        emit rangeChanged(min, max);
    });
}

void PlotView::initializeSeries() {
    struct SeriesInit {
        QString name;
        QColor color;
    };

    const QVector<SeriesInit> seriesInits = {
        {QStringLiteral("S11"), QColor(QString::fromUtf8(S11))},
        {QStringLiteral("S12"), QColor(QString::fromUtf8(S12))},
        {QStringLiteral("S21"), QColor(QString::fromUtf8(S21))},
        {QStringLiteral("S22"), QColor(QString::fromUtf8(S22))}
    };

    for (const auto& init : seriesInits) {
        SeriesSet set;
        set.magnitudeColor = init.color;
        set.phaseColor = init.color;
        set.enabled = true;
        set.magnitudeVisible = true;
        set.phaseVisible = true;

        set.magnitude = new QLineSeries(this);
        set.magnitude->setName(init.name + QStringLiteral(" | Magnitude"));
        set.magnitude->setUseOpenGL(false);
        set.magnitude->setPointsVisible(false);
        set.magnitude->setPen(makePen(set.magnitudeColor, Qt::SolidLine));

        set.phase = new QLineSeries(this);
        set.phase->setName(init.name + QStringLiteral(" | Phase"));
        set.phase->setUseOpenGL(false);
        set.phase->setPointsVisible(false);
        set.phase->setPen(makePen(set.phaseColor, Qt::DashLine));

        chart()->addSeries(set.magnitude);
        chart()->addSeries(set.phase);

        set.magnitude->attachAxis(m_axisFrequency);
        set.magnitude->attachAxis(m_axisMagnitude);

        set.phase->attachAxis(m_axisFrequency);
        set.phase->attachAxis(m_axisPhase);

        m_series.insert(init.name, set);
    }
}

void PlotView::updateSeriesVisibility(const QString& name) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    const bool magnitudeOn = it->enabled && it->magnitudeVisible && m_showMagnitude;
    const bool phaseOn = it->enabled && it->phaseVisible && m_showPhase;
    if (it->magnitude) {
        it->magnitude->setVisible(magnitudeOn);
    }
    if (it->phase) {
        it->phase->setVisible(phaseOn);
    }
}

void PlotView::updateGlobalVisibility() {
    for (auto it = m_series.begin(); it != m_series.end(); ++it) {
        updateSeriesVisibility(it.key());
    }
}

void PlotView::updateAxisTitles() {
    auto textFor = [this](const QString& key, const QString& fallback) {
        return m_strings.value(key, fallback);
    };

    if (m_axisMagnitude) {
        m_axisMagnitude->setTitleText(textFor(QStringLiteral("plot.axis.magnitude"),
                                              QStringLiteral("Magnitude (dB)")));
    }
    if (m_axisPhase) {
        m_axisPhase->setTitleText(textFor(QStringLiteral("plot.axis.phase"),
                                          QStringLiteral("Phase (deg)")));
    }
    if (m_axisFrequency) {
        m_axisFrequency->setTitleText(textFor(QStringLiteral("plot.axis.frequency"),
                                              QStringLiteral("Frequency (GHz)")));
    }
}

void PlotView::refreshSeriesPens(const QString& name) {
    auto it = m_series.find(name);
    if (it == m_series.end()) {
        return;
    }
    if (it->magnitude) {
        it->magnitude->setPen(makePen(it->magnitudeColor, Qt::SolidLine));
    }
    if (it->phase) {
        it->phase->setPen(makePen(it->phaseColor, Qt::DashLine));
    }
}

void PlotView::setFrequencyAxisRange(double minGHz, double maxGHz) {
    if (m_axisFrequency) {
        m_axisFrequency->setRange(minGHz, maxGHz);
    }
}

void PlotView::setMagnitudeAxisRange(double minDb, double maxDb) {
    if (m_axisMagnitude) {
        m_axisMagnitude->setRange(minDb, maxDb);
    }
}

void PlotView::setPhaseAxisRange(double minDeg, double maxDeg) {
    if (m_axisPhase) {
        m_axisPhase->setRange(minDeg, maxDeg);
    }
}
void PlotView::clampFrequencyRange() {
    if (!m_axisFrequency) {
        return;
    }
    const double min = std::max(0.0, m_axisFrequency->min());
    const double max = std::max(min + 0.001, m_axisFrequency->max());
    if (!qFuzzyCompare(min, m_axisFrequency->min()) || !qFuzzyCompare(max, m_axisFrequency->max())) {
        m_axisFrequency->blockSignals(true);
        m_axisFrequency->setRange(min, max);
        m_axisFrequency->blockSignals(false);
    }
}

bool PlotView::handleGesture(QGestureEvent* event) {
    if (!event) {
        return false;
    }
    if (QGesture* gesture = event->gesture(Qt::PinchGesture)) {
        handlePinchGesture(static_cast<QPinchGesture*>(gesture));
        event->accept(gesture);
        return true;
    }
    if (QGesture* pan = event->gesture(Qt::PanGesture)) {
        handlePanGesture(static_cast<QPanGesture*>(pan));
        event->accept(pan);
        return true;
    }
    if (QGesture* tap = event->gesture(Qt::TapGesture)) {
        handleTapGesture(static_cast<QTapGesture*>(tap));
        event->accept(tap);
        return true;
    }
    return false;
}

void PlotView::handlePinchGesture(QPinchGesture* pinch) {
    if (!pinch) {
        return;
    }
    if (pinch->state() == Qt::GestureStarted) {
        m_pinchActive = true;
        if (m_axisFrequency) {
            m_pinchStartFreqMin = m_axisFrequency->min();
            m_pinchStartFreqMax = m_axisFrequency->max();
        }
        if (m_axisMagnitude) {
            m_pinchStartMagMin = m_axisMagnitude->min();
            m_pinchStartMagMax = m_axisMagnitude->max();
        }
        if (m_axisPhase) {
            m_pinchStartPhaseMin = m_axisPhase->min();
            m_pinchStartPhaseMax = m_axisPhase->max();
        }
        return;
    }

    if (!m_pinchActive) {
        return;
    }

    const double scaleFactor = pinch->totalScaleFactor();
    if (!std::isfinite(scaleFactor) || scaleFactor <= 0.0) {
        return;
    }

    applyAxisZoom(m_axisFrequency, m_pinchStartFreqMin, m_pinchStartFreqMax, scaleFactor,
                  0.001, 1e6, true);
    applyAxisZoom(m_axisMagnitude, m_pinchStartMagMin, m_pinchStartMagMax, scaleFactor,
                  0.1, 2000.0);
    applyAxisZoom(m_axisPhase, m_pinchStartPhaseMin, m_pinchStartPhaseMax, scaleFactor,
                  5.0, 720.0);

    clampFrequencyRange();

    if (pinch->state() == Qt::GestureFinished || pinch->state() == Qt::GestureCanceled) {
        m_pinchActive = false;
    }
}

void PlotView::handlePanGesture(QPanGesture* pan) {
    if (!pan) {
        return;
    }
    if (pan->state() == Qt::GestureStarted) {
        m_panActive = true;
        if (m_axisFrequency) {
            m_panStartFreqMin = m_axisFrequency->min();
            m_panStartFreqMax = m_axisFrequency->max();
        }
        if (m_axisMagnitude) {
            m_panStartMagMin = m_axisMagnitude->min();
            m_panStartMagMax = m_axisMagnitude->max();
        }
        if (m_axisPhase) {
            m_panStartPhaseMin = m_axisPhase->min();
            m_panStartPhaseMax = m_axisPhase->max();
        }
        return;
    }

    if (!m_panActive) {
        return;
    }

    const QRectF plotArea = chart() ? chart()->plotArea() : QRectF();
    const double width = std::max(plotArea.width(), 1.0);
    const double height = std::max(plotArea.height(), 1.0);
    const QPointF offset = pan->offset();

    if (m_axisFrequency) {
        const double freqSpan = m_panStartFreqMax - m_panStartFreqMin;
        const double shiftFreq = -offset.x() * (freqSpan / width);
        double newMin = m_panStartFreqMin + shiftFreq;
        double newMax = m_panStartFreqMax + shiftFreq;
        if (newMin < 0.0) {
            const double offset = -newMin;
            newMin += offset;
            newMax += offset;
        }
        m_axisFrequency->setRange(newMin, newMax);
    }

    if (m_axisMagnitude) {
        const double magSpan = m_panStartMagMax - m_panStartMagMin;
        const double shiftMag = offset.y() * (magSpan / height);
        m_axisMagnitude->setRange(m_panStartMagMin + shiftMag, m_panStartMagMax + shiftMag);
    }

    if (m_axisPhase) {
        const double phaseSpan = m_panStartPhaseMax - m_panStartPhaseMin;
        const double shiftPhase = offset.y() * (phaseSpan / height);
        m_axisPhase->setRange(m_panStartPhaseMin + shiftPhase, m_panStartPhaseMax + shiftPhase);
    }

    clampFrequencyRange();

    if (pan->state() == Qt::GestureFinished || pan->state() == Qt::GestureCanceled) {
        m_panActive = false;
    }
}

void PlotView::handleTapGesture(QTapGesture* tap) {
    if (!tap || tap->state() != Qt::GestureFinished) {
        return;
    }
    constexpr qint64 kDoubleTapIntervalMs = 350;
    if (!m_tapTimer.isValid() || m_tapTimer.elapsed() > kDoubleTapIntervalMs) {
        m_tapTimer.start();
        m_tapCount = 1;
        return;
    }

    ++m_tapCount;
    if (m_tapCount >= 2) {
        resetZoom();
        m_tapTimer.invalidate();
        m_tapCount = 0;
    }
}

void PlotView::applyAxisZoom(QValueAxis* axis,
                             double startMin,
                             double startMax,
                             double scaleFactor,
                             double minSpan,
                             double maxSpan,
                             bool enforceNonNegative) {
    if (!axis) {
        return;
    }
    const double spanStart = startMax - startMin;
    if (!(spanStart > 0.0)) {
        return;
    }
    const double clampedMinSpan = std::max(minSpan, 1e-6);
    const double clampedScale = std::clamp(scaleFactor, 0.05, 20.0);

    double span = spanStart / clampedScale;
    span = std::clamp(span, clampedMinSpan, maxSpan);
    double center = (startMin + startMax) * 0.5;

    double newMin = center - span * 0.5;
    double newMax = center + span * 0.5;

    if (enforceNonNegative && newMin < 0.0) {
        const double offset = -newMin;
        newMin += offset;
        newMax += offset;
    }

    if (newMin >= newMax) {
        newMax = newMin + clampedMinSpan;
    }

    axis->setRange(newMin, newMax);
}

void PlotView::resetZoom() {
    if (m_axisFrequency) {
        m_axisFrequency->setRange(m_defaultFreqMin, m_defaultFreqMax);
    }
    if (m_axisMagnitude) {
        m_axisMagnitude->setRange(m_defaultMagMin, m_defaultMagMax);
    }
    if (m_axisPhase) {
        m_axisPhase->setRange(m_defaultPhaseMin, m_defaultPhaseMax);
    }
    clampFrequencyRange();
}

} // namespace ui::widgets





