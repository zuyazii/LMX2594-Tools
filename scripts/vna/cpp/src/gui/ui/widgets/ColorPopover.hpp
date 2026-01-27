// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include <QColor>
#include <QWidget>

class QLineEdit;
class QKeyEvent;
class QHideEvent;

namespace ui::widgets {

namespace detail {
class ColorPlaneWidget;
class HueSliderWidget;
class SaturationSliderWidget;
class ValueSliderWidget;
} // namespace detail

class ColorPopover : public QWidget {
    Q_OBJECT

public:
    explicit ColorPopover(QWidget* parent = nullptr);

    void openFor(QWidget* anchor, const QColor& initial);
    QColor color() const;

signals:
    void colorSelected(const QColor& color);
    void closed();

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void hideEvent(QHideEvent* event) override;

private:
    void buildUi();
    void syncEditors();
    void updatePreview();
    void updateFromHex();
    void updateFromRgb();
    void updateFromPlane(double saturation, double value);
    void updateFromHue(double hue);
    void updateFromSaturation(double saturation);
    void updateFromValue(double value);

    QColor m_color;
    bool m_blockSignals = false;
    QWidget* m_preview = nullptr;
    detail::ColorPlaneWidget* m_colorField = nullptr;
    detail::HueSliderWidget* m_hueSlider = nullptr;
    QLineEdit* m_hexEdit = nullptr;
    QLineEdit* m_rEdit = nullptr;
    QLineEdit* m_gEdit = nullptr;
    QLineEdit* m_bEdit = nullptr;
    detail::SaturationSliderWidget* m_saturationSlider = nullptr;
    detail::ValueSliderWidget* m_valueSlider = nullptr;
};

} // namespace ui::widgets
