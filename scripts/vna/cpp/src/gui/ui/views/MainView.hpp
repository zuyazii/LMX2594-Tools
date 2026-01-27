// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include "../widgets/PlotView.hpp"
#include "BottomPanel.hpp"
#include "Sidebar.hpp"

#include "librevna_headless/device_discovery.hpp"
#include "librevna_headless/host_core.hpp"

#include <QHash>
#include <QLocale>
#include <QWidget>

#include <atomic>
#include <filesystem>
#include <optional>
#include <thread>
#include <vector>

class QSplitter;
class QToolButton;
class QScrollArea;

namespace ui::views {

class MainView : public QWidget {
    Q_OBJECT

public:
    explicit MainView(QWidget* parent = nullptr);
    ~MainView() override;

    void setSeriesColor(const QString& name, const QColor& color);
    void setSeriesVisible(const QString& name, bool visible);
    void setShowMagnitude(bool on);
    void setShowPhase(bool on);

    void setFrequencyRange(double startGHz, double endGHz);
    void setPointCount(int points);
    void setThreshold(const QString& name, double value);

signals:
    void startRequested();
    void stopRequested();
    void resetRequested();
    void rangeChanged(double startGHz, double endGHz);
    void pointsChanged(int points);
    void seriesToggled(const QString& name, bool on);
    void seriesMagnitudeColorChanged(const QString& name, const QColor& color);
    void seriesPhaseColorChanged(const QString& name, const QColor& color);
    void thresholdChanged(const QString& name, double value);
    void magnitudeShown(bool on);
    void phaseShown(bool on);
    void languageChanged(const QLocale& locale);

private:
    void buildUi();
    void setupConnections();
    void initSeriesState();
    void applyLocale(const QLocale& locale);
    void applyTranslations(const QHash<QString, QString>& strings);
    void updateVisibilityUi();
    void updateColorsUi();
    void toggleSidebarVisibility();
    void toggleBottomPanelVisibility();
    void scanForDevices();
    void handleDeviceSelection(int index);
    void handleDeviceActivation(int index);
    void connectToDevice(int index);
    void handleCalibrationSelection(int index);
    void handleCalibrationActivation(int index);
    void loadCalibrationFromFile();
    void applyCalibrationFromPath(const QString& path);
    void refreshCalibrationList();
    void ensureSweepThreadFinished();
    void startSweep();
    void stopSweep();
    void resetSweepParameters();
    void applySweepResults(const std::vector<librevna::headless::VNAMeasurement>& results);
    std::filesystem::path findCalibrationDirectory() const;
    QHash<QString, double> currentThresholds() const;
    void updateSweepControlState(bool running);
    void showInformation(const QString& title, const QString& message);
    void showWarning(const QString& title, const QString& message);

    QHash<QString, QString> loadStrings(const QString& path) const;
    QString resolveLocaleKey(const QLocale& locale) const;
    void onLanguageSelected(const QLocale& locale);

    int m_lastSidebarWidth = 260;
    bool m_sidebarCollapsed = false;
    bool m_bottomCollapsed = false;

    Sidebar* m_sidebar = nullptr;
    QScrollArea* m_sidebarScroll = nullptr;
    BottomPanel* m_bottomPanel = nullptr;
    QWidget* m_bottomWrapper = nullptr;
    widgets::PlotView* m_plot = nullptr;
    QSplitter* m_mainSplitter = nullptr;
    QToolButton* m_sidebarToggle = nullptr;
    QToolButton* m_bottomToggle = nullptr;

    QHash<QString, bool> m_seriesEnabled;
    QHash<QString, bool> m_seriesMagnitudeState;
    QHash<QString, bool> m_seriesPhaseState;
    QHash<QString, QColor> m_seriesMagnitudeColors;
    QHash<QString, QColor> m_seriesPhaseColors;
    QHash<QString, double> m_thresholdValues;
    double m_startFrequencyGHz = 1.0;
    double m_stopFrequencyGHz = 6.0;
    int m_pointCount = 201;

    librevna::headless::HostCore m_hostCore;
    std::vector<librevna::headless::DiscoveredDevice> m_discoveredDevices;
    std::vector<std::filesystem::path> m_calibrationFiles;
    int m_selectedDeviceIndex = -1;
    int m_selectedCalibrationIndex = -1;
    QString m_connectedSerial;
    std::filesystem::path m_calibrationDirectory;
    std::filesystem::path m_activeCalibrationPath;
    std::thread m_sweepThread;
    std::atomic<bool> m_cancelRequested{false};
    std::atomic<bool> m_sweepInProgress{false};

    QLocale m_currentLocale{QLocale::English};
    QHash<QString, QString> m_baseStrings;
    QHash<QString, QString> m_activeStrings;
};

} // namespace ui::views
