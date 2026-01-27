#include "MainView.hpp"

#include "../../ui/theme/DesignTokens.hpp"

#include <QBoxLayout>
#include <QHBoxLayout>
#include <QFile>
#include <QHash>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QCoreApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QDir>
#include <QMetaObject>
#include <limits>
#include <cmath>
#include <chrono>
#include <ctime>
#include <QScroller>
#include <QSplitter>
#include <QToolButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QSize>
#include <QSizePolicy>
#include <QSet>
#include <QStringList>
#include <QVector>
#include <algorithm>
#include <numeric>
#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
#include <vector>
#include <utility>
#include <filesystem>

#include <nlohmann/json.hpp>

namespace ui::views {

namespace fs = std::filesystem;

namespace {

const QStringList kSeriesNames = {
    QStringLiteral("S11"),
    QStringLiteral("S12"),
    QStringLiteral("S21"),
    QStringLiteral("S22")
};

constexpr double kDefaultThresholdDb = -20.0;
constexpr double kFloorMagnitudeDb = -300.0;

struct ParameterOutcome {
    QString name;
    double worstDb = kFloorMagnitudeDb;
    double failFrequencyHz = 0.0;
    bool pass = true;
    double thresholdDb = kDefaultThresholdDb;
    bool hasData = false;
};

struct SweepEvaluationSummary {
    bool overallPass = true;
    QVector<ParameterOutcome> outcomes;
};

std::map<std::string, double> toStdThresholdMap(const QHash<QString, double>& thresholds,
                                                const QStringList& selected) {
    std::map<std::string, double> converted;
    if (!selected.isEmpty()) {
        for (const QString& key : selected) {
            converted.emplace(key.toStdString(), thresholds.value(key, kDefaultThresholdDb));
        }
    } else {
        for (auto it = thresholds.constBegin(); it != thresholds.constEnd(); ++it) {
            converted.emplace(it.key().toStdString(), it.value());
        }
    }

    if (converted.empty()) {
        for (const QString& series : kSeriesNames) {
            converted.emplace(series.toStdString(), thresholds.value(series, kDefaultThresholdDb));
        }
    }
    return converted;
}

std::vector<std::string> toStdStringVector(const QStringList& list) {
    std::vector<std::string> converted;
    converted.reserve(list.size());
    for (const QString& entry : list) {
        converted.push_back(entry.toStdString());
    }
    return converted;
}

SweepEvaluationSummary computeSweepSummary(
    const std::vector<librevna::headless::VNAMeasurement>& measurements,
    const std::map<std::string, double>& thresholdDbByParameter,
    const std::vector<std::string>& selectedParameters) {

    SweepEvaluationSummary summary;
    summary.overallPass = true;

    QVector<QString> parameterOrder;
    if (!selectedParameters.empty()) {
        for (const auto& entry : selectedParameters) {
            parameterOrder.append(QString::fromStdString(entry));
        }
    } else if (!thresholdDbByParameter.empty()) {
        for (const auto& entry : thresholdDbByParameter) {
            parameterOrder.append(QString::fromStdString(entry.first));
        }
    } else {
        parameterOrder = kSeriesNames;
    }

    QSet<QString> seen;
    for (const QString& name : parameterOrder) {
        if (seen.contains(name)) {
            continue;
        }
        seen.insert(name);
        ParameterOutcome outcome;
        outcome.name = name;
        const auto it = thresholdDbByParameter.find(name.toStdString());
        outcome.thresholdDb = it != thresholdDbByParameter.end() ? it->second : kDefaultThresholdDb;
        summary.outcomes.append(outcome);
    }

    for (const auto& measurement : measurements) {
        for (ParameterOutcome& outcome : summary.outcomes) {
            const std::string key = outcome.name.toStdString();
            auto valueIt = measurement.parameters.find(key);
            if (valueIt == measurement.parameters.end()) {
                continue;
            }

            outcome.hasData = true;
            const std::complex<double> value = valueIt->second;
            const double magnitude = std::abs(value);
            const double magnitudeDb =
                magnitude <= 0.0 ? kFloorMagnitudeDb : 20.0 * std::log10(magnitude);

            if (magnitudeDb > outcome.worstDb) {
                outcome.worstDb = magnitudeDb;
                outcome.failFrequencyHz = measurement.frequency;
            }

            if (magnitudeDb > outcome.thresholdDb) {
                outcome.pass = false;
                summary.overallPass = false;
            }
        }
    }

    return summary;
}

fs::path ensureCalibrationStorageRoot() {
    try {
        const fs::path appDir =
            fs::path(QCoreApplication::applicationDirPath().toStdString());
        if (appDir.empty()) {
            return {};
        }
        const fs::path calibrationDir = appDir / "Calibration";
        std::error_code ec;
        fs::create_directories(calibrationDir, ec);
        if (ec && !fs::exists(calibrationDir)) {
            return {};
        }
        const auto canonical = fs::weakly_canonical(calibrationDir, ec);
        return ec ? calibrationDir : canonical;
    } catch (...) {
        return {};
    }
}

fs::path ensureLogsStorageRoot() {
    try {
        const fs::path appDir =
            fs::path(QCoreApplication::applicationDirPath().toStdString());
        if (appDir.empty()) {
            return {};
        }
        const fs::path logsDir = appDir / "Logs";
        std::error_code ec;
        fs::create_directories(logsDir, ec);
        if (ec && !fs::exists(logsDir)) {
            return {};
        }
        const auto canonical = fs::weakly_canonical(logsDir, ec);
        return ec ? logsDir : canonical;
    } catch (...) {
        return {};
    }
}

std::string makeSweepTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &tt);
#else
    localtime_r(&tt, &local);
#endif
    std::ostringstream oss;
    oss << std::put_time(&local, "%Y%m%d_%H%M%S");
    return oss.str();
}

struct SweepArtifactSaveResult {
    bool success = false;
    fs::path directory;
    std::string error;
};

std::vector<std::string> orderedParameterNames(
    const std::vector<librevna::headless::VNAMeasurement>& results) {
    std::vector<std::string> ordered;
    ordered.reserve(kSeriesNames.size());
    std::unordered_set<std::string> seen;
    for (const QString& entry : kSeriesNames) {
        const std::string key = entry.toStdString();
        ordered.push_back(key);
        seen.insert(key);
    }

    std::set<std::string> extras;
    for (const auto& measurement : results) {
        for (const auto& param : measurement.parameters) {
            if (seen.insert(param.first).second) {
                extras.insert(param.first);
            }
        }
    }
    ordered.insert(ordered.end(), extras.begin(), extras.end());
    return ordered;
}

nlohmann::json buildSweepPayload(
    const librevna::headless::SweepConfiguration& configuration,
    const SweepEvaluationSummary& summary,
    const std::vector<librevna::headless::VNAMeasurement>& results,
    const std::string& timestamp) {
    nlohmann::json payload;
    payload["timestamp"] = timestamp;
    payload["configuration"] = {
        {"start_frequency_hz", configuration.start_frequency_hz},
        {"stop_frequency_hz", configuration.stop_frequency_hz},
        {"points", configuration.points},
        {"if_bandwidth_hz", configuration.if_bandwidth_hz},
        {"power_dbm", configuration.power_dbm},
        {"timeout_ms", configuration.timeout_ms},
        {"excited_ports", configuration.excited_ports}};

    nlohmann::json summaryJson;
    summaryJson["overall_pass"] = summary.overallPass;
    nlohmann::json parameters = nlohmann::json::array();
    for (const ParameterOutcome& outcome : summary.outcomes) {
        nlohmann::json entry;
        entry["name"] = outcome.name.toStdString();
        entry["pass"] = outcome.pass;
        entry["has_data"] = outcome.hasData;
        entry["threshold_db"] = outcome.thresholdDb;
        if (outcome.hasData) {
            entry["worst_db"] = outcome.worstDb;
            entry["fail_frequency_hz"] = outcome.failFrequencyHz;
        } else {
            entry["worst_db"] = nullptr;
            entry["fail_frequency_hz"] = nullptr;
        }
        parameters.push_back(std::move(entry));
    }
    summaryJson["parameters"] = std::move(parameters);
    payload["summary"] = std::move(summaryJson);

    const double kRadToDeg = 180.0 / std::acos(-1.0);
    nlohmann::json trace = nlohmann::json::array();
    for (const auto& measurement : results) {
        nlohmann::json entry;
        entry["frequency_hz"] = measurement.frequency;
        nlohmann::json paramsJson = nlohmann::json::object();
        for (const auto& param : measurement.parameters) {
            const std::complex<double> value = param.second;
            const double magnitude = std::abs(value);
            const double magnitudeDb =
                magnitude <= 0.0 ? kFloorMagnitudeDb : 20.0 * std::log10(magnitude);
            const double phaseDeg = std::atan2(value.imag(), value.real()) * kRadToDeg;
            paramsJson[param.first] = {
                {"real", value.real()},
                {"imag", value.imag()},
                {"magnitude_db", magnitudeDb},
                {"phase_deg", phaseDeg}};
        }
        entry["parameters"] = std::move(paramsJson);
        trace.push_back(std::move(entry));
    }
    payload["trace"] = std::move(trace);
    return payload;
}

bool writeJsonFile(const fs::path& path, const nlohmann::json& payload, std::string& error) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        error = "Failed to open JSON file: " + path.string();
        return false;
    }
    stream << payload.dump(2);
    if (!stream.good()) {
        error = "Failed to finish writing JSON file: " + path.string();
        return false;
    }
    return true;
}

bool writeCsvFile(const fs::path& path,
                  const std::vector<librevna::headless::VNAMeasurement>& results,
                  const std::vector<std::string>& orderedNames,
                  std::string& error) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        error = "Failed to open CSV file: " + path.string();
        return false;
    }

    stream << "Frequency (Hz)";
    for (const auto& name : orderedNames) {
        stream << ',' << name << "_Real";
        stream << ',' << name << "_Imag";
        stream << ',' << name << "_Magnitude_dB";
        stream << ',' << name << "_Phase_deg";
    }
    stream << '\n';

    const double kRadToDeg = 180.0 / std::acos(-1.0);
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream << std::setprecision(6);
    for (const auto& measurement : results) {
        stream << measurement.frequency;
        for (const auto& name : orderedNames) {
            const auto it = measurement.parameters.find(name);
            if (it == measurement.parameters.end()) {
                stream << ",,,,"; // Empty columns for missing parameter data.
                continue;
            }
            const std::complex<double> value = it->second;
            const double magnitude = std::abs(value);
            const double magnitudeDb =
                magnitude <= 0.0 ? kFloorMagnitudeDb : 20.0 * std::log10(magnitude);
            const double phaseDeg = std::atan2(value.imag(), value.real()) * kRadToDeg;
            stream << ',' << value.real();
            stream << ',' << value.imag();
            stream << ',' << magnitudeDb;
            stream << ',' << phaseDeg;
        }
        stream << '\n';
    }

    if (!stream.good()) {
        error = "Failed to finish writing CSV file: " + path.string();
        return false;
    }
    return true;
}

SweepArtifactSaveResult persistSweepArtifacts(
    const librevna::headless::SweepConfiguration& configuration,
    const SweepEvaluationSummary& summary,
    const std::vector<librevna::headless::VNAMeasurement>& results) {
    SweepArtifactSaveResult outcome;
    if (results.empty()) {
        outcome.error = "No sweep samples were captured.";
        return outcome;
    }

    const fs::path logsRoot = ensureLogsStorageRoot();
    if (logsRoot.empty()) {
        outcome.error = "Unable to locate or create the Logs folder near the build output.";
        return outcome;
    }

    const std::string timestamp = makeSweepTimestamp();
    const fs::path sweepDir = logsRoot / ("sweep_" + timestamp);
    std::error_code ec;
    fs::create_directories(sweepDir, ec);
    if (ec) {
        outcome.error =
            "Failed to create sweep directory '" + sweepDir.string() + "': " + ec.message();
        return outcome;
    }

    const auto payload = buildSweepPayload(configuration, summary, results, timestamp);
    const auto parameterNames = orderedParameterNames(results);
    std::string error;
    if (!writeJsonFile(sweepDir / "sweep.json", payload, error)) {
        outcome.error = std::move(error);
        return outcome;
    }
    if (!writeCsvFile(sweepDir / "sweep.csv", results, parameterNames, error)) {
        outcome.error = std::move(error);
        return outcome;
    }

    outcome.success = true;
    outcome.directory = sweepDir;
    return outcome;
}

struct CalibrationPlacementResult {
    fs::path path;
    bool storedInManagedDirectory = false;
    std::string warning;
};

CalibrationPlacementResult storeCalibrationFileInManagedDirectory(const fs::path& sourcePath) {
    CalibrationPlacementResult result;
    result.path = sourcePath;
    if (sourcePath.empty() || !fs::exists(sourcePath)) {
        result.warning = "Calibration file does not exist.";
        return result;
    }

    const fs::path storageRoot = ensureCalibrationStorageRoot();
    if (storageRoot.empty()) {
        result.warning = "Unable to access the Calibration folder for storing files.";
        return result;
    }

    std::error_code eq;
    if (fs::equivalent(sourcePath.parent_path(), storageRoot, eq) && !eq) {
        result.storedInManagedDirectory = true;
        result.path = sourcePath;
        return result;
    }

    fs::path destination = storageRoot / sourcePath.filename();
    if (fs::exists(destination)) {
        const std::string timestamp = makeSweepTimestamp();
        const std::string uniqueName =
            sourcePath.stem().string() + "_" + timestamp + sourcePath.extension().string();
        destination = storageRoot / uniqueName;
    }

    std::error_code copyError;
    fs::copy_file(sourcePath, destination, fs::copy_options::overwrite_existing, copyError);
    if (copyError) {
        result.warning =
            "Failed to copy calibration file into Calibration folder: " + copyError.message();
        return result;
    }

    result.path = destination;
    result.storedInManagedDirectory = true;
    return result;
}

QToolButton* makeGhostButton(const QString& text, QWidget* parent) {
    auto* button = new QToolButton(parent);
    button->setText(text);
    button->setCheckable(false);
    button->setAutoRaise(false);
    button->setMinimumSize(120, 32);
    button->setToolButtonStyle(Qt::ToolButtonTextOnly);
    button->setProperty("variant", "ghost");
    return button;
}

} // namespace

MainView::MainView(QWidget* parent)
    : QWidget(parent) {
    setObjectName(QStringLiteral("AppRoot"));
    setAttribute(Qt::WA_StyledBackground, true);

    m_baseStrings = loadStrings(QStringLiteral(":/ui/i18n/en.json"));

    buildUi();
    setupConnections();
    initSeriesState();
    m_calibrationDirectory = findCalibrationDirectory();
    refreshCalibrationList();
    QTimer::singleShot(0, this, &MainView::scanForDevices);
    onLanguageSelected(m_currentLocale);
}

MainView::~MainView() {
    m_cancelRequested = true;
    ensureSweepThreadFinished();
    if (m_hostCore.is_connected()) {
        m_hostCore.disconnect();
    }
}

void MainView::setSeriesColor(const QString& name, const QColor& color) {
    if (!color.isValid()) {
        return;
    }
    m_seriesMagnitudeColors.insert(name, color);
    m_seriesPhaseColors.insert(name, color);
    updateColorsUi();
}

void MainView::setSeriesVisible(const QString& name, bool visible) {
    m_seriesEnabled.insert(name, visible);
    m_seriesMagnitudeState.insert(name, visible);
    m_seriesPhaseState.insert(name, visible);
    updateVisibilityUi();
    m_plot->setSeriesVisible(name, visible);
    m_plot->setSeriesMagnitudeVisible(name, visible);
    m_plot->setSeriesPhaseVisible(name, visible);
}

void MainView::setShowMagnitude(bool on) {
    m_plot->setShowMagnitude(on);
    m_bottomPanel->setMagnitudeVisible(on);
}

void MainView::setShowPhase(bool on) {
    m_plot->setShowPhase(on);
    m_bottomPanel->setPhaseVisible(on);
}

void MainView::setFrequencyRange(double startGHz, double endGHz) {
    m_sidebar->setFrequencyRange(startGHz, endGHz);
}

void MainView::setPointCount(int points) {
    m_sidebar->setPointCount(points);
}

void MainView::setThreshold(const QString& name, double value) {
    m_sidebar->setThresholdValue(name, value);
}

void MainView::buildUi() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    m_mainSplitter = new QSplitter(Qt::Horizontal, this);
    m_mainSplitter->setChildrenCollapsible(false);
    m_mainSplitter->setObjectName(QStringLiteral("MainSplitter"));

    m_sidebarScroll = new QScrollArea(m_mainSplitter);
    m_sidebarScroll->setObjectName(QStringLiteral("SidebarScroll"));
    m_sidebarScroll->setFrameShape(QFrame::NoFrame);
    m_sidebarScroll->setWidgetResizable(true);
    m_sidebarScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_sidebarScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_sidebarScroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    m_sidebar = new Sidebar;
    m_sidebarScroll->setWidget(m_sidebar);
    m_sidebarScroll->setMinimumWidth(m_sidebar->minimumWidth());
    m_sidebarScroll->setMaximumWidth(m_sidebar->maximumWidth());
    if (auto* viewport = m_sidebarScroll->viewport()) {
        viewport->setAttribute(Qt::WA_AcceptTouchEvents, true);
        QScroller::grabGesture(viewport, QScroller::TouchGesture);
        QScroller::grabGesture(viewport, QScroller::LeftMouseButtonGesture);
    }

    auto* rightPane = new QWidget(m_mainSplitter);
    auto* rightPaneLayout = new QVBoxLayout(rightPane);
    rightPaneLayout->setContentsMargins(0, 16, 0, 0);
    rightPaneLayout->setSpacing(16);

    auto* plotSection = new QWidget(rightPane);
    auto* plotLayout = new QVBoxLayout(plotSection);
    plotLayout->setContentsMargins(12, 0, 0, 0);
    plotLayout->setSpacing(0);

    auto* plotTopRow = new QHBoxLayout;
    plotTopRow->setContentsMargins(0, 0, 0, 0);
    plotTopRow->setSpacing(8);

    m_sidebarToggle = new QToolButton(plotSection);
    m_sidebarToggle->setIcon(QIcon(QStringLiteral(":/ui/theme/icons/sidebar_toggle.svg")));
    m_sidebarToggle->setToolTip(tr("Toggle Sidebar"));
    m_sidebarToggle->setCheckable(false);
    m_sidebarToggle->setAutoRaise(false);
    m_sidebarToggle->setFixedSize(32, 32);
    m_sidebarToggle->setIconSize(QSize(18, 18));
    m_sidebarToggle->setToolButtonStyle(Qt::ToolButtonIconOnly);
    m_sidebarToggle->setProperty("variant", "ghost");
    plotTopRow->addWidget(m_sidebarToggle, 0, Qt::AlignLeft);

    plotTopRow->addStretch(1);

    plotLayout->addLayout(plotTopRow);

    m_plot = new widgets::PlotView(plotSection);
    m_plot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    plotLayout->addWidget(m_plot, 1);

    rightPaneLayout->addWidget(plotSection, 1);

    m_bottomWrapper = new QWidget(rightPane);
    m_bottomWrapper->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* bottomLayout = new QVBoxLayout(m_bottomWrapper);
    bottomLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setSpacing(12);

    auto* bottomHeader = new QHBoxLayout;
    bottomHeader->setContentsMargins(0, 0, 0, 0);
    bottomHeader->setSpacing(8);

    bottomHeader->addStretch(1);

    m_bottomToggle = new QToolButton(m_bottomWrapper);
    m_bottomToggle->setIcon(QIcon(QStringLiteral(":/ui/theme/icons/bottom_toggle.svg")));
    m_bottomToggle->setToolTip(tr("Toggle Bottom Panel"));
    m_bottomToggle->setFixedSize(32, 32);
    m_bottomToggle->setIconSize(QSize(14, 14));
    m_bottomToggle->setToolButtonStyle(Qt::ToolButtonIconOnly);
    m_bottomToggle->setProperty("variant", "ghost");
    bottomHeader->addWidget(m_bottomToggle, 0, Qt::AlignRight);

    bottomLayout->addLayout(bottomHeader);

    m_bottomPanel = new BottomPanel(m_bottomWrapper);
    bottomLayout->addWidget(m_bottomPanel);

    rightPaneLayout->addWidget(m_bottomWrapper, 0);

    m_mainSplitter->addWidget(m_sidebarScroll);
    m_mainSplitter->addWidget(rightPane);
    m_mainSplitter->setStretchFactor(0, 0);
    m_mainSplitter->setStretchFactor(1, 1);
    m_mainSplitter->setCollapsible(0, true);

    layout->addWidget(m_mainSplitter);
}

void MainView::setupConnections() {
    connect(m_sidebar, &Sidebar::startRequested, this, &MainView::startRequested);
    connect(m_sidebar, &Sidebar::startRequested, this, &MainView::startSweep);
    connect(m_sidebar, &Sidebar::stopRequested, this, &MainView::stopRequested);
    connect(m_sidebar, &Sidebar::stopRequested, this, &MainView::stopSweep);
    connect(m_sidebar, &Sidebar::resetRequested, this, &MainView::resetRequested);
    connect(m_sidebar, &Sidebar::resetRequested, this, &MainView::resetSweepParameters);

    connect(m_sidebar, &Sidebar::vnaScanRequested, this, &MainView::scanForDevices);
    connect(m_sidebar, &Sidebar::calibrationUploadRequested, this, &MainView::loadCalibrationFromFile);
    connect(m_sidebar, &Sidebar::deviceSelectionChanged, this, &MainView::handleDeviceSelection);
    connect(m_sidebar, &Sidebar::deviceActivated, this, &MainView::handleDeviceActivation);
    connect(m_sidebar, &Sidebar::calibrationSelectionChanged, this, &MainView::handleCalibrationSelection);
    connect(m_sidebar, &Sidebar::calibrationActivated, this, &MainView::handleCalibrationActivation);

    connect(m_sidebar, &Sidebar::rangeChanged, this, [this](double start, double end) {
        m_startFrequencyGHz = start;
        m_stopFrequencyGHz = end;
        emit rangeChanged(start, end);
    });
    connect(m_sidebar, &Sidebar::pointsChanged, this, [this](int points) {
        m_pointCount = points;
        emit pointsChanged(points);
    });

    connect(m_sidebar, &Sidebar::parameterToggled, this, [this](const QString& name, bool on) {
        m_seriesEnabled.insert(name, on);
        updateVisibilityUi();
        m_plot->setSeriesVisible(name, on);
        emit seriesToggled(name, on);
    });

    connect(m_sidebar, &Sidebar::thresholdChanged, this, [this](const QString& name, double value) {
        m_thresholdValues.insert(name, value);
        emit thresholdChanged(name, value);
    });

    connect(m_plot, &widgets::PlotView::rangeChanged, this,
            [this](double start, double end) { emit rangeChanged(start, end); });
    connect(m_bottomPanel, &BottomPanel::languageChanged, this,
            [this](const QLocale& locale) { onLanguageSelected(locale); });

    connect(m_bottomPanel, &BottomPanel::magnitudeColorChanged, this,
            [this](const QString& name, const QColor& color) {
                if (!color.isValid()) {
                    return;
                }
                m_seriesMagnitudeColors.insert(name, color);
                m_plot->setSeriesMagnitudeColor(name, color);
                emit seriesMagnitudeColorChanged(name, color);
            });

    connect(m_bottomPanel, &BottomPanel::phaseColorChanged, this,
            [this](const QString& name, const QColor& color) {
                if (!color.isValid()) {
                    return;
                }
                m_seriesPhaseColors.insert(name, color);
                m_plot->setSeriesPhaseColor(name, color);
                emit seriesPhaseColorChanged(name, color);
            });

    connect(m_bottomPanel, &BottomPanel::magnitudeAllToggled, this, [this](bool on) {
        m_plot->setShowMagnitude(on);
        emit magnitudeShown(on);
    });

    connect(m_bottomPanel, &BottomPanel::phaseAllToggled, this, [this](bool on) {
        m_plot->setShowPhase(on);
        emit phaseShown(on);
    });

    connect(m_bottomPanel, &BottomPanel::seriesMagnitudeToggled, this,
            [this](const QString& name, bool on) {
                m_seriesMagnitudeState.insert(name, on);
                const bool allowed = m_seriesEnabled.value(name, true);
                m_plot->setSeriesMagnitudeVisible(name, allowed && on);
            });

    connect(m_bottomPanel, &BottomPanel::seriesPhaseToggled, this,
            [this](const QString& name, bool on) {
                m_seriesPhaseState.insert(name, on);
                const bool allowed = m_seriesEnabled.value(name, true);
                m_plot->setSeriesPhaseVisible(name, allowed && on);
            });

    connect(m_sidebarToggle, &QToolButton::clicked, this, &MainView::toggleSidebarVisibility);
    connect(m_bottomToggle, &QToolButton::clicked, this, &MainView::toggleBottomPanelVisibility);
}

void MainView::initSeriesState() {
    m_seriesEnabled.clear();
    m_seriesMagnitudeState.clear();
    m_seriesPhaseState.clear();
    m_seriesMagnitudeColors.clear();
    m_seriesPhaseColors.clear();

    m_seriesMagnitudeColors.insert(QStringLiteral("S11"), QColor(QString::fromUtf8(ui::theme::Tokens::S11)));
    m_seriesMagnitudeColors.insert(QStringLiteral("S12"), QColor(QString::fromUtf8(ui::theme::Tokens::S12)));
    m_seriesMagnitudeColors.insert(QStringLiteral("S21"), QColor(QString::fromUtf8(ui::theme::Tokens::S21)));
    m_seriesMagnitudeColors.insert(QStringLiteral("S22"), QColor(QString::fromUtf8(ui::theme::Tokens::S22)));

    m_seriesPhaseColors = m_seriesMagnitudeColors;

    m_thresholdValues.clear();

    for (const QString& name : kSeriesNames) {
        m_seriesEnabled.insert(name, true);
        m_seriesMagnitudeState.insert(name, true);
        m_seriesPhaseState.insert(name, true);
        m_thresholdValues.insert(name, kDefaultThresholdDb);
        m_plot->setSeriesMagnitudeColor(name, m_seriesMagnitudeColors.value(name));
        m_plot->setSeriesPhaseColor(name, m_seriesPhaseColors.value(name));
        m_plot->setSeriesVisible(name, true);
        m_plot->setSeriesMagnitudeVisible(name, true);
        m_plot->setSeriesPhaseVisible(name, true);
    }

    updateVisibilityUi();
    updateColorsUi();
    updateSweepControlState(false);
    m_bottomPanel->resetResultStates(kSeriesNames);
}

void MainView::applyLocale(const QLocale& locale) {
    QLocale::setDefault(locale);
    m_sidebar->applyLocale(locale);
    m_bottomPanel->applyLocale(locale);
}

void MainView::applyTranslations(const QHash<QString, QString>& strings) {
    m_activeStrings = strings;
    m_sidebar->applyTranslations(m_activeStrings);
    m_bottomPanel->applyTranslations(m_activeStrings);
    m_plot->applyTranslations(m_activeStrings);

    m_sidebarToggle->setToolTip(
        m_activeStrings.value(QStringLiteral("panel.toggleSidebar"), tr("Toggle Sidebar")));
    m_bottomToggle->setToolTip(
        m_activeStrings.value(QStringLiteral("panel.toggleBottom"), tr("Toggle Bottom Panel")));
}

void MainView::updateVisibilityUi() {
    m_bottomPanel->setSeriesEnabled(m_seriesEnabled);
    m_bottomPanel->setMagnitudeState(m_seriesMagnitudeState);
    m_bottomPanel->setPhaseState(m_seriesPhaseState);
    m_sidebar->setSeriesSelection(m_seriesEnabled);

    for (auto it = m_seriesMagnitudeState.constBegin(); it != m_seriesMagnitudeState.constEnd(); ++it) {
        const bool allowed = m_seriesEnabled.value(it.key(), true);
        m_plot->setSeriesMagnitudeVisible(it.key(), allowed && it.value());
    }
    for (auto it = m_seriesPhaseState.constBegin(); it != m_seriesPhaseState.constEnd(); ++it) {
        const bool allowed = m_seriesEnabled.value(it.key(), true);
        m_plot->setSeriesPhaseVisible(it.key(), allowed && it.value());
    }
}

void MainView::updateColorsUi() {
    auto defaultColorFor = [](const QString& series) -> QColor {
        if (series == QStringLiteral("S12")) {
            return QColor(QString::fromUtf8(ui::theme::Tokens::S12));
        }
        if (series == QStringLiteral("S21")) {
            return QColor(QString::fromUtf8(ui::theme::Tokens::S21));
        }
        if (series == QStringLiteral("S22")) {
            return QColor(QString::fromUtf8(ui::theme::Tokens::S22));
        }
        return QColor(QString::fromUtf8(ui::theme::Tokens::S11));
    };

    auto phaseVariantFor = [](const QColor& base) -> QColor {
        QColor variant = base.lighter(115);
        if (variant == base) {
            variant = base.darker(115);
        }
        variant.setAlpha(255);
        return variant;
    };

    for (const QString& name : kSeriesNames) {
        QColor magnitudeColor = m_seriesMagnitudeColors.value(name);
        if (!magnitudeColor.isValid()) {
            magnitudeColor = defaultColorFor(name);
            m_seriesMagnitudeColors.insert(name, magnitudeColor);
        }
        QColor phaseColor = m_seriesPhaseColors.value(name);
        if (!phaseColor.isValid()) {
            phaseColor = phaseVariantFor(magnitudeColor);
            m_seriesPhaseColors.insert(name, phaseColor);
        } else if (phaseColor == magnitudeColor) {
            phaseColor = phaseVariantFor(magnitudeColor);
            m_seriesPhaseColors.insert(name, phaseColor);
        }
        m_plot->setSeriesMagnitudeColor(name, magnitudeColor);
        m_plot->setSeriesPhaseColor(name, phaseColor);
    }
    m_bottomPanel->setSeriesColors(m_seriesMagnitudeColors, m_seriesPhaseColors);
}

void MainView::toggleSidebarVisibility() {
    if (!m_mainSplitter) {
        return;
    }
    const int sidebarWidth = m_sidebar ? m_sidebar->sizeHint().width() : 280;
    const QList<int> sizes = m_mainSplitter->sizes();
    if (!m_sidebarCollapsed) {
        m_lastSidebarWidth = sidebarWidth;
        QList<int> newSizes{0};
        if (sizes.size() >= 2) {
            newSizes.append(sizes.at(0) + sizes.at(1));
        } else {
            newSizes.append(1);
        }
        m_mainSplitter->setSizes(newSizes);
        if (m_sidebarScroll) {
            m_sidebarScroll->setVisible(false);
            m_sidebarScroll->setMinimumWidth(0);
        }
        m_sidebarCollapsed = true;
        m_sidebarToggle->setIcon(QIcon(QStringLiteral(":/ui/theme/icons/sidebar_toggle.svg")));
    } else {
        int total = std::accumulate(sizes.begin(), sizes.end(), 0);
        if (total <= 0) {
            total = m_mainSplitter->width();
        }
        if (total <= 0) {
            total = 800;
        }
        const int preferred = sidebarWidth;
        QList<int> restored{preferred, std::max(total - preferred, 200)};
        m_mainSplitter->setSizes(restored);
        if (m_sidebarScroll) {
            m_sidebarScroll->setVisible(true);
            m_sidebarScroll->setMinimumWidth(m_sidebar ? m_sidebar->minimumWidth() : preferred);
            m_sidebarScroll->setMaximumWidth(m_sidebar ? m_sidebar->maximumWidth() : preferred);
        }
        m_sidebarCollapsed = false;
        m_sidebarToggle->setIcon(QIcon(QStringLiteral(":/ui/theme/icons/sidebar_toggle.svg")));
    }
}

void MainView::toggleBottomPanelVisibility() {
    if (!m_bottomPanel) {
        return;
    }
    m_bottomCollapsed = !m_bottomCollapsed;
    if (m_bottomPanel) {
        m_bottomPanel->setVisible(!m_bottomCollapsed);
    }
    m_bottomToggle->setIcon(QIcon(QStringLiteral(":/ui/theme/icons/bottom_toggle.svg")));
}

QHash<QString, QString> MainView::loadStrings(const QString& path) const {
    QHash<QString, QString> map;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return map;
    }
    const auto document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject()) {
        return map;
    }
    const auto object = document.object();
    for (auto it = object.begin(); it != object.end(); ++it) {
        if (it.value().isString()) {
            map.insert(it.key(), it.value().toString());
        }
    }
    return map;
}

QString MainView::resolveLocaleKey(const QLocale& locale) const {
    const QString name = locale.name();
    const auto language = locale.language();
    const auto script = locale.script();
    const auto territory = locale.territory();

    const auto isTraditionalRegion = (territory == QLocale::Taiwan || territory == QLocale::HongKong ||
                                      territory == QLocale::Macao);

    if (name.startsWith(QStringLiteral("zh_Hant"), Qt::CaseInsensitive) ||
        (language == QLocale::Chinese &&
         (script == QLocale::TraditionalChineseScript || isTraditionalRegion))) {
        return QStringLiteral("zh_Hant");
    }
    if (name.startsWith(QStringLiteral("zh_Hans"), Qt::CaseInsensitive) ||
        language == QLocale::Chinese) {
        return QStringLiteral("zh_Hans");
    }
    return QStringLiteral("en");
}

void MainView::onLanguageSelected(const QLocale& locale) {
    m_currentLocale = locale;
    applyLocale(locale);

    QHash<QString, QString> strings = m_baseStrings;
    const QString key = resolveLocaleKey(locale);
    if (key != QStringLiteral("en")) {
        const QString overridePath = QStringLiteral(":/ui/i18n/%1.json").arg(key);
        const QHash<QString, QString> overrideMap = loadStrings(overridePath);
        for (auto it = overrideMap.constBegin(); it != overrideMap.constEnd(); ++it) {
            strings.insert(it.key(), it.value());
        }
    }

    applyTranslations(strings);
    emit languageChanged(locale);
}
void MainView::scanForDevices() {
    std::string errorMessage;
    auto devices = librevna::headless::discover_devices(errorMessage);

    if (!errorMessage.empty()) {
        showWarning(tr("Device Scan Failed"),
                    tr("Unable to enumerate LibreVNA devices.\n%1")
                        .arg(QString::fromStdString(errorMessage)));
    }

    m_discoveredDevices = std::move(devices);

    QStringList deviceNames;
    deviceNames.reserve(static_cast<int>(m_discoveredDevices.size()));
    for (const auto& device : m_discoveredDevices) {
        const QString label = QString::fromStdString(device.label.empty() ? std::string("LibreVNA") : device.label);
        const QString serial = device.serial.empty() ? tr("<no-serial>")
                                                     : QString::fromStdString(device.serial);
        deviceNames.append(QStringLiteral("%1 (%2)").arg(label, serial));
    }

    m_sidebar->setDevices(deviceNames);

    if (!m_discoveredDevices.empty()) {
        m_sidebar->setSelectedDevice(0);
        handleDeviceSelection(0);
    } else {
        m_sidebar->setSelectedDevice(-1);
        handleDeviceSelection(-1);
        showInformation(tr("Device Scan"), tr("No LibreVNA devices detected."));
    }
}

void MainView::handleDeviceSelection(int index) {
    m_selectedDeviceIndex = index;
}

void MainView::handleDeviceActivation(int index) {
    connectToDevice(index);
}

void MainView::connectToDevice(int index) {
    if (index < 0 || index >= static_cast<int>(m_discoveredDevices.size())) {
        showWarning(tr("Connection"), tr("Select a device before attempting to connect."));
        return;
    }

    ensureSweepThreadFinished();

    const auto& device = m_discoveredDevices[static_cast<std::size_t>(index)];
    const QString label = QString::fromStdString(device.label.empty() ? std::string("LibreVNA") : device.label);
    const QString serial = device.serial.empty() ? tr("<no-serial>")
                                                 : QString::fromStdString(device.serial);

    if (m_hostCore.is_connected()) {
        m_hostCore.disconnect();
    }

    if (!m_hostCore.connect(device.serial)) {
        const QString message = QString::fromStdString(m_hostCore.last_error_message());
        showWarning(tr("Connection Failed"),
                    tr("Unable to connect to %1 (%2).\n%3").arg(label, serial, message));
        return;
    }

    m_connectedSerial = serial;
    showInformation(tr("Device Connected"),
                    tr("Connected to %1 (%2). The system is ready for calibration.").arg(label, serial));
}

void MainView::handleCalibrationSelection(int index) {
    m_selectedCalibrationIndex = index;
}

void MainView::handleCalibrationActivation(int index) {
    if (index < 0 || index >= static_cast<int>(m_calibrationFiles.size())) {
        return;
    }
    const fs::path& path = m_calibrationFiles[static_cast<std::size_t>(index)];
    applyCalibrationFromPath(QString::fromStdString(path.u8string()));
}


void MainView::loadCalibrationFromFile() {
    QString startDir;
    if (!m_calibrationDirectory.empty()) {
        startDir = QString::fromStdString(m_calibrationDirectory.u8string());
    } else {
        startDir = QDir::currentPath();
    }

    const QString selected = QFileDialog::getOpenFileName(
        this,
        tr("Select Calibration File"),
        startDir,
        tr("Calibration Files (*.cal);;All Files (*.*)"));

    if (selected.isEmpty()) {
        return;
    }

    applyCalibrationFromPath(selected);
}

void MainView::applyCalibrationFromPath(const QString& path) {
    if (path.isEmpty()) {
        return;
    }

    const fs::path calPath = fs::path(path.toStdString());
    if (!fs::exists(calPath)) {
        showWarning(tr("Calibration"), tr("The selected calibration file no longer exists."));
        return;
    }

    const auto placement = storeCalibrationFileInManagedDirectory(calPath);
    fs::path pathToLoad = placement.path.empty() ? calPath : placement.path;
    if (!placement.warning.empty()) {
        showWarning(tr("Calibration Storage"),
                    tr("The calibration file could not be cached for reuse.\n%1")
                        .arg(QString::fromStdString(placement.warning)));
    }

    if (!m_hostCore.load_calibration(pathToLoad)) {
        const QString message = QString::fromStdString(m_hostCore.last_error_message());
        showWarning(tr("Calibration Failed"),
                    tr("Unable to load calibration file.\n%1").arg(message));
        return;
    }

    m_activeCalibrationPath = pathToLoad;
    const fs::path parent = pathToLoad.parent_path();
    if (!parent.empty() && fs::exists(parent) && fs::is_directory(parent)) {
        std::error_code ec;
        const auto canonicalParent = fs::weakly_canonical(parent, ec);
        m_calibrationDirectory = ec ? parent : canonicalParent;
    }

    refreshCalibrationList();
    const QString storedPath =
        QDir::toNativeSeparators(QString::fromStdString(pathToLoad.u8string()));
    showInformation(tr("Calibration Loaded"), storedPath);
}

void MainView::refreshCalibrationList() {
    QStringList entries;
    m_calibrationFiles.clear();

    if (m_calibrationDirectory.empty() || !fs::exists(m_calibrationDirectory)) {
        m_sidebar->setCalibrations(entries);
        m_sidebar->setSelectedCalibration(-1);
        return;
    }

    std::vector<fs::directory_entry> files;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(m_calibrationDirectory, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto extension = QString::fromStdString(entry.path().extension().string()).toLower();
        if (extension != QStringLiteral(".cal")) {
            continue;
        }
        files.push_back(entry);
    }

    std::sort(files.begin(), files.end(), [](const fs::directory_entry& lhs, const fs::directory_entry& rhs) {
        return lhs.path().filename().string() < rhs.path().filename().string();
    });

    int activeIndex = -1;
    for (std::size_t i = 0; i < files.size(); ++i) {
        const auto& entry = files[i];
        entries.append(QString::fromStdString(entry.path().filename().string()));
        m_calibrationFiles.push_back(entry.path());
        if (!m_activeCalibrationPath.empty()) {
            std::error_code compareError;
            if (fs::equivalent(entry.path(), m_activeCalibrationPath, compareError) && !compareError) {
                activeIndex = static_cast<int>(i);
            }
        }
    }

    m_sidebar->setCalibrations(entries);
    if (activeIndex >= 0) {
        m_sidebar->setSelectedCalibration(activeIndex);
        m_selectedCalibrationIndex = activeIndex;
    } else if (!entries.isEmpty()) {
        m_sidebar->setSelectedCalibration(0);
        m_selectedCalibrationIndex = 0;
    } else {
        m_sidebar->setSelectedCalibration(-1);
        m_selectedCalibrationIndex = -1;
    }
}

void MainView::ensureSweepThreadFinished() {
    if (m_sweepThread.joinable()) {
        if (m_sweepThread.get_id() == std::this_thread::get_id()) {
            m_sweepThread.detach();
        } else {
            m_sweepThread.join();
        }
    }
    m_sweepThread = std::thread();
    m_sweepInProgress = false;
}

void MainView::startSweep() {
    if (m_sweepInProgress) {
        showInformation(tr("Sweep"), tr("A sweep is already in progress."));
        return;
    }

    if (!m_hostCore.is_connected()) {
        showWarning(tr("Sweep"), tr("Connect to a LibreVNA device before starting a sweep."));
        return;
    }

    if (m_activeCalibrationPath.empty()) {
        showWarning(tr("Sweep"), tr("Load a calibration file before starting a sweep."));
        return;
    }

    QStringList activeParameters;
    for (auto it = m_seriesEnabled.constBegin(); it != m_seriesEnabled.constEnd(); ++it) {
        if (it.value()) {
            activeParameters.append(it.key());
        }
    }
    if (activeParameters.isEmpty()) {
        showWarning(tr("Sweep"), tr("Select at least one S-parameter before starting the sweep."));
        return;
    }

    if (m_stopFrequencyGHz <= m_startFrequencyGHz) {
        showWarning(tr("Sweep"), tr("Stop frequency must be greater than start frequency."));
        return;
    }

    librevna::headless::SweepConfiguration configuration;
    configuration.start_frequency_hz = m_startFrequencyGHz * 1e9;
    configuration.stop_frequency_hz = m_stopFrequencyGHz * 1e9;
    configuration.points = static_cast<std::uint32_t>(m_pointCount);
    configuration.if_bandwidth_hz = 1000.0;
    configuration.power_dbm = 0.0;
    configuration.timeout_ms = 15000.0;
    configuration.excited_ports = {1, 2};

    const QHash<QString, double> thresholds = currentThresholds();
    const QStringList parameters = activeParameters;
    const auto thresholdMap = toStdThresholdMap(thresholds, parameters);
    const auto selectedStdParameters = toStdStringVector(parameters);

    m_cancelRequested = false;
    m_sweepInProgress = true;
    updateSweepControlState(true);
    {
        QHash<QString, BottomPanel::ResultState> initialStates;
        QSet<QString> activeSet;
        if (!parameters.isEmpty()) {
            activeSet.reserve(parameters.size());
            for (const QString& entry : parameters) {
                activeSet.insert(entry);
            }
        }
        const bool filter = !parameters.isEmpty();
        for (const QString& name : kSeriesNames) {
            const bool enabled = !filter || activeSet.contains(name);
            initialStates.insert(name,
                                 enabled ? BottomPanel::ResultState::Pending
                                         : BottomPanel::ResultState::Disabled);
        }
        m_bottomPanel->setResultStates(initialStates);
    }

    m_sweepThread = std::thread([this,
                                 configuration,
                                 thresholdMap,
                                 selectedStdParameters,
                                 parameters]() mutable {
        auto results = m_hostCore.run_sweep(configuration);
        const std::string lastError = m_hostCore.last_error_message();
        const bool cancelled = m_cancelRequested.load();
        const std::size_t resultCount = results.size();
        const auto summary = computeSweepSummary(results, thresholdMap, selectedStdParameters);

        QMetaObject::invokeMethod(
            this,
            [this,
             cancelled,
             lastError,
             configuration,
             summary,
             resultCount,
             selectedParameters = parameters,
             results = std::move(results)]() mutable {
                if (m_sweepThread.joinable()) {
                    m_sweepThread.join();
                    m_sweepThread = std::thread();
                }

                m_sweepInProgress = false;
                m_cancelRequested = false;
                updateSweepControlState(false);

                const double startGHz = configuration.start_frequency_hz / 1e9;
                const double stopGHz = configuration.stop_frequency_hz / 1e9;
                const QLocale locale = m_currentLocale;
                const QString startText = locale.toString(startGHz, 'f', 3);
                const QString stopText = locale.toString(stopGHz, 'f', 3);

                if (cancelled) {
                    showInformation(tr("Sweep"),
                                    tr("Sweep from %1 GHz to %2 GHz cancelled.")
                                        .arg(startText, stopText));
                    return;
                }

                if (resultCount == 0) {
                    const QString message = lastError.empty()
                                                ? tr("No data returned from the sweep.")
                                                : QString::fromStdString(lastError);
                    m_bottomPanel->resetResultStates(selectedParameters);
                    showWarning(tr("Sweep"), message);
                    return;
                }

                applySweepResults(results);
                {
                    QHash<QString, BottomPanel::ResultState> finalStates;
                    const bool filter = !selectedParameters.isEmpty();
                    QSet<QString> activeSet;
                    if (!selectedParameters.isEmpty()) {
                        activeSet.reserve(selectedParameters.size());
                        for (const QString& entry : selectedParameters) {
                            activeSet.insert(entry);
                        }
                    }
                    for (const QString& name : kSeriesNames) {
                        if (filter && !activeSet.contains(name)) {
                            finalStates.insert(name, BottomPanel::ResultState::Disabled);
                            continue;
                        }
                        const auto outcomeIt = std::find_if(
                            summary.outcomes.cbegin(), summary.outcomes.cend(),
                            [&name](const ParameterOutcome& outcome) { return outcome.name == name; });
                        if (outcomeIt == summary.outcomes.cend() || !outcomeIt->hasData) {
                            finalStates.insert(name, BottomPanel::ResultState::NoData);
                        } else if (outcomeIt->pass) {
                            finalStates.insert(name, BottomPanel::ResultState::Passed);
                        } else {
                            finalStates.insert(name, BottomPanel::ResultState::Failed);
                        }
                    }
                    m_bottomPanel->setResultStates(finalStates);
                }

                const QString pointsText =
                    locale.toString(static_cast<qulonglong>(resultCount));

                QString message =
                    tr("Sweep %1 GHz -> %2 GHz (%3 points).").arg(startText, stopText, pointsText);
                QStringList lines;
                for (const ParameterOutcome& outcome : summary.outcomes) {
                    if (!selectedParameters.isEmpty() && !selectedParameters.contains(outcome.name)) {
                        continue;
                    }
                    QString status = outcome.pass ? tr("PASS") : tr("FAIL");
                    QString line;
                    if (!outcome.hasData) {
                        line = tr("%1: no data received.").arg(outcome.name);
                    } else {
                        const QString worstDb = locale.toString(outcome.worstDb, 'f', 2);
                        const QString freqGHz =
                            locale.toString(outcome.failFrequencyHz / 1e9, 'f', 3);
                        const QString thresholdDb =
                            locale.toString(outcome.thresholdDb, 'f', 2);
                        line = tr("%1: %2 (worst %3 dB @ %4 GHz, threshold %5 dB)")
                                   .arg(outcome.name, status, worstDb, freqGHz, thresholdDb);
                    }
                    lines.append(QStringLiteral("- %1").arg(line));
                }

                if (!lines.isEmpty()) {
                    message.append(QLatin1String("\n\n"));
                    message.append(tr("Summary:"));
                    message.append(QLatin1Char('\n'));
                    message.append(lines.join(QLatin1Char('\n')));
                }

                const auto storageResult = persistSweepArtifacts(configuration, summary, results);
                if (storageResult.success && !storageResult.directory.empty()) {
                    message.append(QLatin1String("\n\n"));
                    message.append(
                        tr("Sweep data saved to %1.")
                            .arg(QDir::toNativeSeparators(
                                QString::fromStdString(storageResult.directory.u8string()))));
                } else if (!storageResult.error.empty()) {
                    message.append(QLatin1String("\n\n"));
                    message.append(
                        tr("Failed to store sweep data: %1")
                            .arg(QString::fromStdString(storageResult.error)));
                }

                if (summary.overallPass) {
                    showInformation(tr("Sweep Complete"), message);
                } else {
                    showWarning(tr("Sweep Complete"), message);
                }
            },
            Qt::QueuedConnection);
    });
}

void MainView::stopSweep() {
    if (!m_sweepInProgress) {
        return;
    }
    m_cancelRequested = true;
    showInformation(tr("Sweep"), tr("Stop requested. The sweep will halt shortly."));
}

void MainView::resetSweepParameters() {
    if (m_sweepInProgress) {
        return;
    }

    m_sidebar->setFrequencyRange(1.0, 6.0);
    m_sidebar->setPointCount(201);
    m_startFrequencyGHz = 1.0;
    m_stopFrequencyGHz = 6.0;
    m_pointCount = 201;

    QHash<QString, bool> selection;
    for (const QString& name : kSeriesNames) {
        selection.insert(name, true);
        m_seriesEnabled.insert(name, true);
        m_seriesMagnitudeState.insert(name, true);
        m_seriesPhaseState.insert(name, true);
    }
    m_sidebar->setSeriesSelection(selection);
    m_bottomPanel->setMagnitudeState(selection);
    m_bottomPanel->setPhaseState(selection);

    QHash<QString, double> defaults;
    for (const QString& name : kSeriesNames) {
        defaults.insert(name, kDefaultThresholdDb);
        m_thresholdValues.insert(name, kDefaultThresholdDb);
    }
    m_sidebar->setThresholds(defaults);
    m_bottomPanel->resetResultStates(kSeriesNames);

    for (const QString& name : kSeriesNames) {
        m_plot->setSeriesData(name, {}, {});
    }

    updateVisibilityUi();
    updateColorsUi();
    updateSweepControlState(false);
}

void MainView::applySweepResults(const std::vector<librevna::headless::VNAMeasurement>& results) {
    if (results.empty()) {
        return;
    }

    QHash<QString, QVector<QPointF>> magnitudePoints;
    QHash<QString, QVector<QPointF>> phasePoints;

    double frequencyMin = std::numeric_limits<double>::max();
    double frequencyMax = std::numeric_limits<double>::lowest();
    double magnitudeMin = std::numeric_limits<double>::max();
    double magnitudeMax = std::numeric_limits<double>::lowest();
    double phaseMin = std::numeric_limits<double>::max();
    double phaseMax = std::numeric_limits<double>::lowest();

    const double kRadToDeg = 180.0 / std::acos(-1.0);

    for (const auto& measurement : results) {
        const double frequencyGHz = measurement.frequency / 1e9;
        frequencyMin = std::min(frequencyMin, frequencyGHz);
        frequencyMax = std::max(frequencyMax, frequencyGHz);

        for (const auto& entry : measurement.parameters) {
            const QString key = QString::fromStdString(entry.first);
            const std::complex<double> value = entry.second;
            const double magnitude = std::abs(value);
            const double magnitudeDb =
                magnitude <= 0.0 ? kFloorMagnitudeDb : 20.0 * std::log10(magnitude);
            const double phaseDeg = std::atan2(value.imag(), value.real()) * kRadToDeg;

            auto& magVec = magnitudePoints[key];
            magVec.append(QPointF(frequencyGHz, magnitudeDb));
            auto& phaseVec = phasePoints[key];
            phaseVec.append(QPointF(frequencyGHz, phaseDeg));

            magnitudeMin = std::min(magnitudeMin, magnitudeDb);
            magnitudeMax = std::max(magnitudeMax, magnitudeDb);
            phaseMin = std::min(phaseMin, phaseDeg);
            phaseMax = std::max(phaseMax, phaseDeg);
        }
    }

    for (const QString& name : kSeriesNames) {
        const QVector<QPointF> magVec = magnitudePoints.value(name);
        const QVector<QPointF> phaseVec = phasePoints.value(name);
        m_plot->setSeriesData(name, magVec, phaseVec);
    }

    if (frequencyMin < frequencyMax) {
        m_plot->setFrequencyAxisRange(frequencyMin, frequencyMax);
    }
    if (magnitudeMin < magnitudeMax) {
        const double padding = 3.0;
        m_plot->setMagnitudeAxisRange(magnitudeMin - padding, magnitudeMax + padding);
    }
    if (phaseMin < phaseMax) {
        const double padding = 10.0;
        m_plot->setPhaseAxisRange(phaseMin - padding, phaseMax + padding);
    }
}

std::filesystem::path MainView::findCalibrationDirectory() const {
    if (const fs::path preferred = ensureCalibrationStorageRoot(); !preferred.empty()) {
        return preferred;
    }

    std::vector<fs::path> candidates;

    try {
        const fs::path appDir = fs::path(QCoreApplication::applicationDirPath().toStdString());
        candidates.push_back(appDir / "Calibration");
        candidates.push_back(appDir / "calibration");
        candidates.push_back(appDir.parent_path() / "calibration");
    } catch (...) {
        // Ignore failures retrieving application directory.
    }

    candidates.push_back(fs::current_path() / "Calibration");
    candidates.push_back(fs::current_path() / "calibration");

    for (const auto& candidate : candidates) {
        if (!candidate.empty() && fs::exists(candidate) && fs::is_directory(candidate)) {
            std::error_code ec;
            const auto canonical = fs::weakly_canonical(candidate, ec);
            return ec ? candidate : canonical;
        }
    }

    return {};
}

QHash<QString, double> MainView::currentThresholds() const {
    return m_thresholdValues;
}

void MainView::updateSweepControlState(bool running) {
    m_sidebar->setSweepControlsEnabled(!running, running, !running);
}

void MainView::showInformation(const QString& title, const QString& message) {
    QMessageBox::information(this, title, message);
}

void MainView::showWarning(const QString& title, const QString& message) {
    QMessageBox::warning(this, title, message);
}

} // namespace ui::views








