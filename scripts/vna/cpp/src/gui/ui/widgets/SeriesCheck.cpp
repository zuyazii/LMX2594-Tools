#include "SeriesCheck.hpp"

#include "../theme/DesignTokens.hpp"

#include <QCheckBox>
#include <QCursor>
#include <QHBoxLayout>
#include <QToolButton>

namespace ui::widgets {

SeriesCheck::SeriesCheck(const QString& seriesName, QWidget* parent)
    : QWidget(parent)
    , m_seriesName(seriesName)
    , m_color(Qt::black) {
    setAttribute(Qt::WA_StyledBackground, true);
    setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    buildUi();
}

void SeriesCheck::setChecked(bool checked) {
    if (m_check) {
        m_check->setChecked(checked);
    }
}

bool SeriesCheck::isChecked() const {
    return m_check && m_check->isChecked();
}

void SeriesCheck::setColor(const QColor& color) {
    if (!color.isValid() || color == m_color) {
        return;
    }
    m_color = color;
    updateIndicator();
}

QColor SeriesCheck::color() const {
    return m_color;
}

QString SeriesCheck::seriesName() const {
    return m_seriesName;
}

void SeriesCheck::buildUi() {
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 2, 0, 2);
    layout->setSpacing(6);

    m_check = new QCheckBox(m_seriesName, this);
    m_check->setMinimumHeight(28);
    m_check->setObjectName(QStringLiteral("SeriesCheckBox"));
    layout->addWidget(m_check);

    m_colorButton = new QToolButton(this);
    m_colorButton->setAccessibleName(tr("%1 Color").arg(m_seriesName));
    m_colorButton->setCheckable(false);
    m_colorButton->setAutoRaise(false);
    m_colorButton->setFixedSize(18, 18);
    m_colorButton->setToolTip(tr("Adjust %1 color").arg(m_seriesName));
    m_colorButton->setProperty("variant", "ghost");
    m_colorButton->setCursor(Qt::PointingHandCursor);
    layout->addWidget(m_colorButton);
    layout->addStretch(1);

    updateIndicator();

    connect(m_check, &QCheckBox::toggled, this, &SeriesCheck::toggled);
    connect(m_colorButton, &QToolButton::clicked, this, [this]() {
        openPopover();
    });
}

void SeriesCheck::updateIndicator() {
    if (!m_colorButton) {
        return;
    }
    const int radius = ui::theme::Tokens::radius(ui::theme::Tokens::Radius::XS);
    QString style = QStringLiteral(
        "QToolButton { "
        "border-radius: %1px;"
        "border: 1px solid %2;"
        "background-color: %3;"
        "min-width: 18px;"
        "min-height: 18px;"
        "max-width: 18px;"
        "max-height: 18px;"
        "}"
        "QToolButton:hover { background-color: %4; }")
                       .arg(radius)
                       .arg(QString::fromUtf8(ui::theme::Tokens::StrokeSoft))
                       .arg(m_color.name(QColor::HexRgb))
                       .arg(m_color.lighter(110).name(QColor::HexRgb));
    m_colorButton->setStyleSheet(style);
}

void SeriesCheck::ensurePopover() {
    if (m_popover) {
        return;
    }
    m_popover = new ColorPopover(this);
    connect(m_popover.data(), &ColorPopover::colorSelected, this, [this](const QColor& c) {
        setColor(c);
        emit colorPicked(c);
    });
    connect(m_popover.data(), &ColorPopover::closed, this, [this]() {
        if (m_colorButton) {
            m_colorButton->setDown(false);
        }
    });
}

void SeriesCheck::openPopover() {
    ensurePopover();
    if (!m_popover) {
        return;
    }
    m_colorButton->setDown(true);
    m_popover->openFor(m_colorButton, m_color);
}

} // namespace ui::widgets
