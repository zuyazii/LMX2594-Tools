#include "ColorPopover.hpp"

#include "../theme/DesignTokens.hpp"

#include <QGridLayout>
#include <QtGlobal>
#include <QHideEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QHBoxLayout>
#include <QImage>
#include <QLinearGradient>
#include <QLineEdit>
#include <QIntValidator>
#include <QMouseEvent>
#include <QPainter>
#include <QScreen>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace ui::widgets {

namespace detail {

constexpr double kEpsilon = 1e-6;

inline double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

QColor colorFromHsv(double hue, double saturation, double value, double alpha) {
    QColor c;
    c.setHsvF(clamp01(hue), clamp01(saturation), clamp01(value), clamp01(alpha));
    return c;
}

class ColorPlaneWidget : public QWidget {
    Q_OBJECT

public:
    explicit ColorPlaneWidget(QWidget* parent = nullptr)
        : QWidget(parent) {
        constexpr int kDimension = 220;
        setFixedSize(kDimension, kDimension);
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    }

    void setHue(double hue) {
        hue = clamp01(hue);
        if (std::abs(m_hue - hue) < kEpsilon) {
            return;
        }
        m_hue = hue;
        m_dirty = true;
        update();
    }

    void setPoint(double saturation, double value) {
        saturation = clamp01(saturation);
        value = clamp01(value);
        if (std::abs(m_saturation - saturation) < kEpsilon &&
            std::abs(m_value - value) < kEpsilon) {
            return;
        }
        m_saturation = saturation;
        m_value = value;
        update();
    }

    double saturation() const { return m_saturation; }
    double value() const { return m_value; }

signals:
    void picked(double saturation, double value);

protected:
    void paintEvent(QPaintEvent*) override {
        if (m_dirty || m_cache.devicePixelRatio() != devicePixelRatioF() ||
            m_cache.size() !=
                (colorRect().size() * devicePixelRatioF()).toSize()) {
            regenerateCache();
        }

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRectF area = colorRect();
        painter.drawImage(area, m_cache);

        painter.setPen(QPen(QColor(QString::fromUtf8(ui::theme::Tokens::StrokeSoft)), 1.0));
        painter.drawRoundedRect(rect().adjusted(0.5, 0.5, -0.5, -0.5), 10.0, 10.0);

        const QPointF indicatorPos = pointFor(m_saturation, m_value);
        painter.setBrush(Qt::NoBrush);
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(indicatorPos, 8, 8);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(indicatorPos, 8, 8);
    }

    void resizeEvent(QResizeEvent* event) override {
        QWidget::resizeEvent(event);
        m_dirty = true;
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            updateFromPos(event->position());
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton) {
            updateFromPos(event->position());
        }
    }

private:
    QRectF colorRect() const {
        constexpr qreal margin = 2.0;
        const QRectF r = rect();
        return QRectF(r.left() + margin,
                      r.top() + margin,
                      std::max<qreal>(1.0, r.width() - margin * 2.0),
                      std::max<qreal>(1.0, r.height() - margin * 2.0));
    }

    void regenerateCache() {
        const QRectF area = colorRect();
        const qreal dpr = devicePixelRatioF();
        const QSize cacheSize =
            QSizeF(area.size().width() * dpr, area.size().height() * dpr).toSize();
        if (cacheSize.isEmpty()) {
            return;
        }
        m_cache = QImage(cacheSize, QImage::Format_RGB32);
        m_cache.setDevicePixelRatio(dpr);

        const int w = std::max(1, cacheSize.width());
        const int h = std::max(1, cacheSize.height());
        const int wSpan = std::max(1, w - 1);
        const int hSpan = std::max(1, h - 1);

        for (int y = 0; y < h; ++y) {
            const double value = 1.0 - static_cast<double>(y) / hSpan;
            QRgb* line = reinterpret_cast<QRgb*>(m_cache.scanLine(y));
            for (int x = 0; x < w; ++x) {
                const double saturation = static_cast<double>(x) / wSpan;
                const QColor c = colorFromHsv(m_hue, saturation, value, 1.0);
                line[x] = c.rgb();
            }
        }
        m_dirty = false;
    }

    QPointF pointFor(double saturation, double value) const {
        const QRectF area = colorRect();
        const double widthRange = std::max<qreal>(1.0, area.width() - 1.0);
        const double heightRange = std::max<qreal>(1.0, area.height() - 1.0);
        const double x = area.left() + saturation * widthRange;
        const double y = area.top() + (1.0 - value) * heightRange;
        return QPointF(x, y);
    }

    void updateFromPos(const QPointF& pos) {
        const QRectF area = colorRect();
        const double widthRange = std::max<qreal>(1.0, area.width() - 1.0);
        const double heightRange = std::max<qreal>(1.0, area.height() - 1.0);
        const double clampedX =
            std::clamp(pos.x(), area.left(), area.left() + widthRange);
        const double clampedY =
            std::clamp(pos.y(), area.top(), area.top() + heightRange);
        const double saturation = clamp01((clampedX - area.left()) / widthRange);
        const double value = clamp01(1.0 - (clampedY - area.top()) / heightRange);
        setPoint(saturation, value);
        emit picked(saturation, value);
    }

    double m_hue = 0.0;
    double m_saturation = 1.0;
    double m_value = 0.5;
    QImage m_cache;
    bool m_dirty = true;
};

class HueSliderWidget : public QWidget {
    Q_OBJECT

public:
    explicit HueSliderWidget(QWidget* parent = nullptr)
        : QWidget(parent) {
        setFixedHeight(22);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setHue(double hue) {
        hue = clamp01(hue);
        if (std::abs(m_hue - hue) < kEpsilon) {
            return;
        }
        m_hue = hue;
        update();
    }

    double hue() const { return m_hue; }

signals:
    void hueChanged(double hue);

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QLinearGradient gradient(rect().topLeft(), rect().topRight());
        gradient.setColorAt(0.0, QColor::fromHsl(0, 255, 128));
        gradient.setColorAt(1.0 / 6.0, QColor::fromHsl(43, 255, 128));
        gradient.setColorAt(2.0 / 6.0, QColor::fromHsl(85, 255, 128));
        gradient.setColorAt(3.0 / 6.0, QColor::fromHsl(128, 255, 128));
        gradient.setColorAt(4.0 / 6.0, QColor::fromHsl(170, 255, 128));
        gradient.setColorAt(5.0 / 6.0, QColor::fromHsl(213, 255, 128));
        gradient.setColorAt(1.0, QColor::fromHsl(0, 255, 128));

        QRect inner = rect().adjusted(0, 6, 0, -6);
        painter.setPen(Qt::NoPen);
        painter.setBrush(gradient);
        painter.drawRoundedRect(inner, 6, 6);

        painter.setPen(QPen(QColor(QString::fromUtf8(ui::theme::Tokens::StrokeSoft)), 1.0));
        painter.drawRoundedRect(inner, 6, 6);

        const double x = inner.left() + m_hue * inner.width();
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(QPointF(x, rect().center().y()), 7, 7);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(QPointF(x, rect().center().y()), 7, 7);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

private:
    void setFromPosition(const QPointF& pos) {
        const QRect inner = rect().adjusted(0, 6, 0, -6);
        if (inner.width() <= 0) {
            return;
        }
        const double hue = clamp01((pos.x() - inner.left()) / inner.width());
        m_hue = hue;
        emit hueChanged(m_hue);
        update();
    }

    double m_hue = 0.0;
};

class SaturationSliderWidget : public QWidget {
    Q_OBJECT

public:
    explicit SaturationSliderWidget(QWidget* parent = nullptr)
        : QWidget(parent) {
        setFixedHeight(24);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setHue(double hue) {
        hue = clamp01(hue);
        if (std::abs(m_hue - hue) < kEpsilon) {
            return;
        }
        m_hue = hue;
        update();
    }

    void setValue(double value) {
        value = clamp01(value);
        if (std::abs(m_value - value) < kEpsilon) {
            return;
        }
        m_value = value;
        update();
    }

    void setSaturation(double saturation) {
        saturation = clamp01(saturation);
        if (std::abs(m_saturation - saturation) < kEpsilon) {
            return;
        }
        m_saturation = saturation;
        update();
    }

signals:
    void saturationChanged(double saturation);

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QRect inner = rect().adjusted(0, 6, 0, -6);
        QLinearGradient gradient(inner.topLeft(), inner.topRight());
        gradient.setColorAt(0.0, QColor::fromHsvF(m_hue, 0.0, m_value));
        gradient.setColorAt(1.0, QColor::fromHsvF(m_hue, 1.0, m_value));
        painter.setBrush(gradient);
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(inner, 6, 6);

        painter.setPen(QPen(QColor(QString::fromUtf8(ui::theme::Tokens::StrokeSoft)), 1.0));
        painter.drawRoundedRect(inner, 6, 6);

        const double x = inner.left() + m_saturation * inner.width();
        const QPointF pos(x, rect().center().y());
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(pos, 7, 7);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(pos, 7, 7);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

private:
    void setFromPosition(const QPointF& pos) {
        const QRect inner = rect().adjusted(0, 6, 0, -6);
        if (inner.width() <= 0) {
            return;
        }
        const double saturation = clamp01((pos.x() - inner.left()) / inner.width());
        if (std::abs(m_saturation - saturation) < kEpsilon) {
            return;
        }
        m_saturation = saturation;
        emit saturationChanged(m_saturation);
        update();
    }

    double m_hue = 0.0;
    double m_value = 1.0;
    double m_saturation = 1.0;
};

class ValueSliderWidget : public QWidget {
    Q_OBJECT

public:
    explicit ValueSliderWidget(QWidget* parent = nullptr)
        : QWidget(parent) {
        setFixedHeight(24);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setHue(double hue) {
        hue = clamp01(hue);
        if (std::abs(m_hue - hue) < kEpsilon) {
            return;
        }
        m_hue = hue;
        update();
    }

    void setSaturation(double saturation) {
        saturation = clamp01(saturation);
        if (std::abs(m_saturation - saturation) < kEpsilon) {
            return;
        }
        m_saturation = saturation;
        update();
    }

    void setValue(double value) {
        value = clamp01(value);
        if (std::abs(m_value - value) < kEpsilon) {
            return;
        }
        m_value = value;
        update();
    }

signals:
    void valueChanged(double value);

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QRect inner = rect().adjusted(0, 6, 0, -6);
        QLinearGradient gradient(inner.topLeft(), inner.topRight());
        gradient.setColorAt(0.0, QColor::fromHsvF(m_hue, m_saturation, 0.0));
        gradient.setColorAt(1.0, QColor::fromHsvF(m_hue, m_saturation, 1.0));
        painter.setBrush(gradient);
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(inner, 6, 6);

        painter.setPen(QPen(QColor(QString::fromUtf8(ui::theme::Tokens::StrokeSoft)), 1.0));
        painter.drawRoundedRect(inner, 6, 6);

        const double x = inner.left() + m_value * inner.width();
        const QPointF pos(x, rect().center().y());
        painter.setPen(QPen(Qt::white, 2));
        painter.drawEllipse(pos, 7, 7);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(pos, 7, 7);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton) {
            setFromPosition(event->position());
        }
    }

private:
    void setFromPosition(const QPointF& pos) {
        const QRect inner = rect().adjusted(0, 6, 0, -6);
        if (inner.width() <= 0) {
            return;
        }
        const double value = clamp01((pos.x() - inner.left()) / inner.width());
        if (std::abs(m_value - value) < kEpsilon) {
            return;
        }
        m_value = value;
        emit valueChanged(m_value);
        update();
    }

    double m_hue = 0.0;
    double m_saturation = 1.0;
    double m_value = 1.0;
};

} // namespace detail

using detail::SaturationSliderWidget;
using detail::ColorPlaneWidget;
using detail::ValueSliderWidget;
using detail::HueSliderWidget;
using detail::clamp01;
using detail::colorFromHsv;

ColorPopover::ColorPopover(QWidget* parent)
    : QWidget(parent)
    , m_color(Qt::white) {
    setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_StyledBackground, true);
    setFocusPolicy(Qt::StrongFocus);
    setProperty("type", "card");
    buildUi();
    syncEditors();
}

void ColorPopover::openFor(QWidget* anchor, const QColor& initial) {
    if (!anchor) {
        return;
    }
    QColor fallback(QString::fromUtf8(theme::Tokens::S11));
    m_color = initial.isValid() ? initial : fallback;
    syncEditors();
    updatePreview();

    adjustSize();
    QRect geometry = frameGeometry();
    geometry.moveTopLeft(anchor->mapToGlobal(anchor->rect().bottomRight()) + QPoint(8, 8));

    if (const QScreen* screen = anchor->screen()) {
        const QRect available = screen->availableGeometry();
        if (geometry.right() > available.right()) {
            geometry.moveRight(available.right());
        }
        if (geometry.bottom() > available.bottom()) {
            geometry.moveBottom(available.bottom());
        }
        if (geometry.left() < available.left()) {
            geometry.moveLeft(available.left());
        }
        if (geometry.top() < available.top()) {
            geometry.moveTop(available.top());
        }
    }

    setGeometry(geometry);

    show();
    raise();
    activateWindow();
    setFocus(Qt::ActiveWindowFocusReason);
}

QColor ColorPopover::color() const {
    return m_color;
}

void ColorPopover::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        close();
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

void ColorPopover::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    emit closed();
}

void ColorPopover::buildUi() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(12);

    m_preview = new QWidget(this);
    m_preview->setFixedSize(48, 48);
    m_preview->setProperty("type", "card");
    m_preview->setAutoFillBackground(true);

    auto* headerRow = new QHBoxLayout;
    headerRow->setSpacing(12);
    headerRow->addWidget(m_preview, 0, Qt::AlignLeft | Qt::AlignTop);

    auto* codeColumn = new QVBoxLayout;
    codeColumn->setSpacing(8);

    auto* hexRow = new QHBoxLayout;
    hexRow->setSpacing(8);
    auto* hexLabel = new QLabel(tr("HEX"), this);
    hexLabel->setProperty("role", "subtitle");
    hexRow->addWidget(hexLabel);
    m_hexEdit = new QLineEdit(this);
    m_hexEdit->setPlaceholderText(QStringLiteral("#RRGGBB"));
    m_hexEdit->setMaxLength(7);
    hexRow->addWidget(m_hexEdit, 1);
    codeColumn->addLayout(hexRow);

    auto* rgbRow = new QHBoxLayout;
    rgbRow->setSpacing(8);
    auto makeChannelField = [this, rgbRow](const QString& name, QLineEdit** store) {
        auto* label = new QLabel(name, this);
        label->setProperty("role", "subtitle");
        rgbRow->addWidget(label);
        auto* edit = new QLineEdit(this);
        edit->setAlignment(Qt::AlignCenter);
        edit->setFixedWidth(48);
        edit->setMaxLength(3);
        edit->setValidator(new QIntValidator(0, 255, edit));
        rgbRow->addWidget(edit);
        if (store) {
            *store = edit;
        }
    };
    makeChannelField(tr("R"), &m_rEdit);
    makeChannelField(tr("G"), &m_gEdit);
    makeChannelField(tr("B"), &m_bEdit);
    rgbRow->addStretch(1);
    codeColumn->addLayout(rgbRow);

    headerRow->addLayout(codeColumn, 1);
    layout->addLayout(headerRow);

    m_colorField = new ColorPlaneWidget(this);
    layout->addWidget(m_colorField, 0, Qt::AlignHCenter);

    auto makeSliderBlock = [this, layout](const QString& title, QWidget* slider) {
        auto* block = new QVBoxLayout;
        block->setSpacing(4);
        auto* label = new QLabel(title, this);
        label->setProperty("role", "subtitle");
        block->addWidget(label, 0, Qt::AlignLeft);
        block->addWidget(slider);
        layout->addLayout(block);
    };

    m_hueSlider = new HueSliderWidget(this);
    makeSliderBlock(tr("Hue"), m_hueSlider);

    m_saturationSlider = new SaturationSliderWidget(this);
    makeSliderBlock(tr("Saturation"), m_saturationSlider);

    m_valueSlider = new ValueSliderWidget(this);
    makeSliderBlock(tr("Value"), m_valueSlider);

    connect(m_hexEdit, &QLineEdit::editingFinished, this, &ColorPopover::updateFromHex);
    connect(m_rEdit, &QLineEdit::editingFinished, this, &ColorPopover::updateFromRgb);
    connect(m_gEdit, &QLineEdit::editingFinished, this, &ColorPopover::updateFromRgb);
    connect(m_bEdit, &QLineEdit::editingFinished, this, &ColorPopover::updateFromRgb);

    connect(m_colorField, &ColorPlaneWidget::picked, this,
            [this](double s, double v) { updateFromPlane(s, v); });
    connect(m_hueSlider, &HueSliderWidget::hueChanged, this,
            [this](double hue) { updateFromHue(hue); });
    connect(m_saturationSlider, &SaturationSliderWidget::saturationChanged, this,
            [this](double s) { updateFromSaturation(s); });
    connect(m_valueSlider, &ValueSliderWidget::valueChanged, this,
            [this](double v) { updateFromValue(v); });
}

void ColorPopover::syncEditors() {
    m_blockSignals = true;

    const QString hex =
        QStringLiteral("#%1").arg(static_cast<quint32>(m_color.rgb()) & 0xFFFFFFu, 6, 16, QLatin1Char('0')).toUpper();
    if (m_hexEdit) {
        m_hexEdit->setText(hex);
    }

    const int r = m_color.red();
    const int g = m_color.green();
    const int b = m_color.blue();
    if (m_rEdit) {
        m_rEdit->setText(QString::number(r));
    }
    if (m_gEdit) {
        m_gEdit->setText(QString::number(g));
    }
    if (m_bEdit) {
        m_bEdit->setText(QString::number(b));
    }

    double hue = m_color.hsvHueF();
    if (hue < 0.0) {
        hue = 0.0;
    }
    const double saturation = m_color.hsvSaturationF();
    const double value = m_color.valueF();

    if (m_colorField) {
        m_colorField->setHue(hue);
        m_colorField->setPoint(saturation, value);
    }
    if (m_hueSlider) {
        m_hueSlider->setHue(hue);
    }
    if (m_saturationSlider) {
        m_saturationSlider->setHue(hue);
        m_saturationSlider->setValue(value);
        m_saturationSlider->setSaturation(saturation);
    }
    if (m_valueSlider) {
        m_valueSlider->setHue(hue);
        m_valueSlider->setSaturation(saturation);
        m_valueSlider->setValue(value);
    }

    updatePreview();

    m_blockSignals = false;
}

void ColorPopover::updatePreview() {
    if (!m_preview) {
        return;
    }
    const QString border = QString::fromUtf8(theme::Tokens::StrokeSoft);
    const QString color = m_color.name(QColor::HexRgb);
    m_preview->setStyleSheet(
        QStringLiteral("background-color: %1; border: 1px solid %2; border-radius: 12px;")
            .arg(color, border));
}

void ColorPopover::updateFromHex() {
    if (m_blockSignals) {
        return;
    }
    QString text = m_hexEdit->text().trimmed();
    if (text.startsWith('#')) {
        text.remove(0, 1);
    }
    if (text.length() != 6) {
        return;
    }
    bool ok = false;
    const int value = text.toInt(&ok, 16);
    if (!ok) {
        return;
    }
    QColor parsed((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF);
    if (!parsed.isValid()) {
        return;
    }
    m_color = parsed;
    syncEditors();
    emit colorSelected(m_color);
}

void ColorPopover::updateFromRgb() {
    if (m_blockSignals) {
        return;
    }
    auto readChannel = [](QLineEdit* edit, int& out) -> bool {
        if (!edit) {
            return false;
        }
        bool ok = false;
        const int value = edit->text().toInt(&ok);
        if (!ok || value < 0 || value > 255) {
            return false;
        }
        out = value;
        return true;
    };

    int r = 0;
    int g = 0;
    int b = 0;
    if (!readChannel(m_rEdit, r) || !readChannel(m_gEdit, g) || !readChannel(m_bEdit, b)) {
        return;
    }

    QColor parsed(r, g, b);
    if (!parsed.isValid()) {
        return;
    }
    m_color = parsed;
    syncEditors();
    emit colorSelected(m_color);
}

void ColorPopover::updateFromPlane(double saturation, double value) {
    if (m_blockSignals) {
        return;
    }
    const double hue = m_hueSlider ? m_hueSlider->hue() : 0.0;
    m_color = QColor::fromHsvF(hue, clamp01(saturation), clamp01(value));
    syncEditors();
    emit colorSelected(m_color);
}

void ColorPopover::updateFromHue(double hue) {
    if (m_blockSignals) {
        return;
    }
    const double saturation =
        m_colorField ? m_colorField->saturation() : m_color.hsvSaturationF();
    const double value =
        m_colorField ? m_colorField->value() : m_color.valueF();
    m_color = QColor::fromHsvF(clamp01(hue), clamp01(saturation), clamp01(value));
    syncEditors();
    emit colorSelected(m_color);
}

void ColorPopover::updateFromSaturation(double saturation) {
    if (m_blockSignals) {
        return;
    }
    const double hue = m_hueSlider ? m_hueSlider->hue() : m_color.hsvHueF();
    const double value =
        m_colorField ? m_colorField->value() : m_color.valueF();
    const double clampedHue = hue < 0.0 ? 0.0 : hue;
    m_color =
        QColor::fromHsvF(clamp01(clampedHue), clamp01(saturation), clamp01(value));
    syncEditors();
    emit colorSelected(m_color);
}

void ColorPopover::updateFromValue(double value) {
    if (m_blockSignals) {
        return;
    }
    const double hue = m_hueSlider ? m_hueSlider->hue() : m_color.hsvHueF();
    const double saturation =
        m_colorField ? m_colorField->saturation() : m_color.hsvSaturationF();
    const double clampedHue = hue < 0.0 ? 0.0 : hue;
    m_color =
        QColor::fromHsvF(clamp01(clampedHue), clamp01(saturation), clamp01(value));
    syncEditors();
    emit colorSelected(m_color);
}

} // namespace ui::widgets

#include "ColorPopover.moc"

