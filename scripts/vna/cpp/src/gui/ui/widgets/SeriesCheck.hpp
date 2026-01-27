// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include "ColorPopover.hpp"

#include <QColor>
#include <QPointer>
#include <QWidget>

class QCheckBox;
class QToolButton;

namespace ui::widgets {

class SeriesCheck : public QWidget {
    Q_OBJECT

public:
    explicit SeriesCheck(const QString& seriesName, QWidget* parent = nullptr);

    void setChecked(bool checked);
    bool isChecked() const;

    void setColor(const QColor& color);
    QColor color() const;

    QString seriesName() const;

signals:
    void toggled(bool on);
    void colorPicked(const QColor& color);

private:
    void buildUi();
    void updateIndicator();
    void ensurePopover();
    void openPopover();

    QString m_seriesName;
    QColor m_color;
    QCheckBox* m_check = nullptr;
    QToolButton* m_colorButton = nullptr;
    QPointer<ColorPopover> m_popover;
};

} // namespace ui::widgets
