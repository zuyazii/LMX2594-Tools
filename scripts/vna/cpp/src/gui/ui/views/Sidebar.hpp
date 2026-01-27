// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include <QHash>
#include <QLocale>
#include <QStringList>
#include <QWidget>

#include <vector>

class QListView;
class QBoxLayout;
class QVBoxLayout;
class QLabel;
class QGridLayout;
class QStringListModel;
class QPushButton;
class QDoubleSpinBox;
class QSpinBox;
class QToolButton;
class QResizeEvent;

namespace ui::views {

class Sidebar : public QWidget {
    Q_OBJECT

public:
    explicit Sidebar(QWidget* parent = nullptr);

    void setDevices(const QStringList& devices);
    void setCalibrations(const QStringList& calibrations);
    void setSeriesSelection(const QHash<QString, bool>& selection);
    void setThresholds(const QHash<QString, double>& thresholds);
    void setFrequencyRange(double startGHz, double endGHz);
    void setPointCount(int points);
    void setThresholdValue(const QString& name, double value);

    void applyTranslations(const QHash<QString, QString>& strings);
    void applyLocale(const QLocale& locale);
    void setSelectedDevice(int index);
    void setSelectedCalibration(int index);
    void setSweepControlsEnabled(bool startEnabled, bool stopEnabled, bool resetEnabled);

protected:
    void resizeEvent(QResizeEvent* event) override;

signals:
    void startRequested();
    void stopRequested();
    void resetRequested();
    void rangeChanged(double startGHz, double endGHz);
    void pointsChanged(int points);
    void parameterToggled(const QString& name, bool on);
    void thresholdChanged(const QString& name, double value);
    void vnaScanRequested();
    void calibrationUploadRequested();
    void deviceSelectionChanged(int index);
    void deviceActivated(int index);
    void calibrationSelectionChanged(int index);
    void calibrationActivated(int index);

private:
    void buildUi();
    void bindSignals();
    void updateSpinLocale();
    void updateSectionTitles();
    void updateFrequencyLayoutMode(int availableWidth);
    void updateParameterLayoutMode(int availableWidth);
    void updateParameterButtonSizing(int availableWidth);
    QString trKey(const QString& key, const QString& fallback) const;
    void setLabelText(const QString& key, QLabel* label, const QString& fallback);

    struct ParameterRow {
        QWidget* container = nullptr;
        QBoxLayout* controlLayout = nullptr;
        QWidget* inputContainer = nullptr;
        QVBoxLayout* inputLayout = nullptr;
        QPushButton* button = nullptr;
        QDoubleSpinBox* spin = nullptr;
        QLabel* caption = nullptr;
    };

    QListView* m_deviceList = nullptr;
    QListView* m_calibrationList = nullptr;
    QDoubleSpinBox* m_startSpin = nullptr;
    QDoubleSpinBox* m_endSpin = nullptr;
    QSpinBox* m_pointsSpin = nullptr;
    QHash<QString, QPushButton*> m_paramButtons;
    QHash<QString, QDoubleSpinBox*> m_thresholds;
    QHash<QString, QLabel*> m_thresholdCaptions;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_resetButton = nullptr;
    QToolButton* m_vnaScanButton = nullptr;
    QToolButton* m_calibrationUploadButton = nullptr;

    QHash<QString, QString> m_strings;
    QLocale m_locale;

    QLabel* m_vnaLabel = nullptr;
    QLabel* m_calibrationLabel = nullptr;
    QLabel* m_frequencyLabel = nullptr;
    QLabel* m_pointsLabel = nullptr;
    QLabel* m_parametersLabel = nullptr;
    QWidget* m_startFrequencyField = nullptr;
    QWidget* m_endFrequencyField = nullptr;
    QLabel* m_startFrequencyLabel = nullptr;
    QLabel* m_endFrequencyLabel = nullptr;
    QLabel* m_frequencyDash = nullptr;
    QBoxLayout* m_frequencyLayout = nullptr;
    bool m_frequencyStacked = false;
    int m_frequencyWrapThreshold = 0;
    bool m_paramButtonsCompact = false;
    int m_paramCompactThreshold = 0;
    int m_paramButtonTallHeight = 64;
    int m_paramButtonShortHeight = 48;
    bool m_paramStackedLayout = false;
    int m_paramStackThreshold = 520;
    QGridLayout* m_parameterGrid = nullptr;
    std::vector<ParameterRow> m_parameterRows;
    bool m_devicePlaceholderActive = false;
    QString m_devicePlaceholderText;

    QStringListModel* m_deviceModel = nullptr;
    QStringListModel* m_calibrationModel = nullptr;
};

} // namespace ui::views
