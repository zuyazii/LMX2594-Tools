#include "Sidebar.hpp"

#include "../../ui/theme/DesignTokens.hpp"

#include <algorithm>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QBoxLayout>
#include <QCursor>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QItemSelectionModel>
#include <QListView>
#include <QPushButton>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QStringListModel>
#include <QScroller>
#include <QToolButton>
#include <QVBoxLayout>
#include <QSize>

namespace ui::views {

namespace {

constexpr double kStartMin = 0.001;
constexpr double kStartMax = 18.000;
constexpr double kEndMin = 0.001;
constexpr double kEndMax = 40.000;

constexpr double kThresholdMin = -120.0;
constexpr double kThresholdMax = 20.0;

constexpr int kSectionSpacing = 36;
constexpr int kParamCompactWidthThreshold = 420;

} // namespace

Sidebar::Sidebar(QWidget* parent)
    : QWidget(parent) {
    setObjectName(QStringLiteral("Sidebar"));
    setMinimumWidth(280);
    setMaximumWidth(QWIDGETSIZE_MAX);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_StyledBackground, true);
    setAttribute(Qt::WA_AcceptTouchEvents, true);
    setProperty("type", "card");
    m_paramCompactThreshold = kParamCompactWidthThreshold;

    buildUi();
    bindSignals();
}

void Sidebar::setDevices(const QStringList& devices) {
    if (!m_deviceModel) {
        m_deviceModel = new QStringListModel(this);
        m_deviceList->setModel(m_deviceModel);
    }

    QStringList listContents;
    if (devices.isEmpty()) {
        m_devicePlaceholderActive = true;
        if (m_devicePlaceholderText.isEmpty()) {
            m_devicePlaceholderText =
                trKey(QStringLiteral("sidebar.noDevice"), tr("No VNA Detected"));
        }
        listContents.append(m_devicePlaceholderText);
        m_deviceList->setEnabled(false);
    } else {
        m_devicePlaceholderActive = false;
        listContents = devices;
        m_deviceList->setEnabled(true);
    }

    m_deviceModel->setStringList(listContents);
    m_deviceList->setCurrentIndex(m_deviceModel->index(0));
}

void Sidebar::setCalibrations(const QStringList& calibrations) {
    if (!m_calibrationModel) {
        m_calibrationModel = new QStringListModel(this);
        m_calibrationList->setModel(m_calibrationModel);
    }
    m_calibrationModel->setStringList(calibrations);
    if (!calibrations.isEmpty()) {
        m_calibrationList->setCurrentIndex(m_calibrationModel->index(0));
    }
}

void Sidebar::setSeriesSelection(const QHash<QString, bool>& selection) {
    for (auto it = m_paramButtons.begin(); it != m_paramButtons.end(); ++it) {
        QPushButton* button = it.value();
        const bool value = selection.value(it.key(), true);
        QSignalBlocker blocker(button);
        button->setChecked(value);
        if (m_thresholds.contains(it.key())) {
            m_thresholds.value(it.key())->setEnabled(value);
        }
    }
}

void Sidebar::setThresholds(const QHash<QString, double>& thresholds) {
    for (auto it = thresholds.constBegin(); it != thresholds.constEnd(); ++it) {
        if (m_thresholds.contains(it.key())) {
            QDoubleSpinBox* spin = m_thresholds.value(it.key());
            QSignalBlocker blocker(spin);
            spin->setValue(it.value());
        }
    }
}

void Sidebar::setFrequencyRange(double startGHz, double endGHz) {
    if (m_startSpin) {
        m_startSpin->setValue(startGHz);
    }
    if (m_endSpin) {
        m_endSpin->setValue(endGHz);
    }
}

void Sidebar::setPointCount(int points) {
    if (m_pointsSpin) {
        m_pointsSpin->setValue(points);
    }
}

void Sidebar::setThresholdValue(const QString& name, double value) {
    if (m_thresholds.contains(name)) {
        QDoubleSpinBox* spin = m_thresholds.value(name);
        QSignalBlocker blocker(spin);
        spin->setValue(value);
    }
}

void Sidebar::setSelectedDevice(int index) {
    if (!m_deviceList || !m_deviceModel) {
        return;
    }
    if (m_devicePlaceholderActive) {
        return;
    }
    const int rowCount = m_deviceModel->rowCount();
    if (index < 0 || index >= rowCount) {
        if (auto* selection = m_deviceList->selectionModel()) {
            selection->clearSelection();
            selection->setCurrentIndex(QModelIndex(), QItemSelectionModel::Clear);
        }
        return;
    }
    const QModelIndex modelIndex = m_deviceModel->index(index, 0);
    if (!modelIndex.isValid()) {
        return;
    }
    if (auto* selection = m_deviceList->selectionModel()) {
        selection->setCurrentIndex(modelIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }
    m_deviceList->scrollTo(modelIndex, QAbstractItemView::PositionAtCenter);
}

void Sidebar::setSelectedCalibration(int index) {
    if (!m_calibrationList || !m_calibrationModel) {
        return;
    }
    const int rowCount = m_calibrationModel->rowCount();
    if (index < 0 || index >= rowCount) {
        if (auto* selection = m_calibrationList->selectionModel()) {
            selection->clearSelection();
            selection->setCurrentIndex(QModelIndex(), QItemSelectionModel::Clear);
        }
        return;
    }
    const QModelIndex modelIndex = m_calibrationModel->index(index, 0);
    if (!modelIndex.isValid()) {
        return;
    }
    if (auto* selection = m_calibrationList->selectionModel()) {
        selection->setCurrentIndex(modelIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }
    m_calibrationList->scrollTo(modelIndex, QAbstractItemView::PositionAtCenter);
}

void Sidebar::setSweepControlsEnabled(bool startEnabled, bool stopEnabled, bool resetEnabled) {
    if (m_startButton) {
        m_startButton->setEnabled(startEnabled);
    }
    if (m_stopButton) {
        m_stopButton->setEnabled(stopEnabled);
    }
    if (m_resetButton) {
        m_resetButton->setEnabled(resetEnabled);
    }
}

void Sidebar::applyTranslations(const QHash<QString, QString>& strings) {
    m_strings = strings;
    updateSectionTitles();
    for (auto it = m_thresholdCaptions.begin(); it != m_thresholdCaptions.end(); ++it) {
        if (it.value()) {
            it.value()->setText(tr("threshold"));
        }
    }
    m_startButton->setText(trKey(QStringLiteral("actions.start"), tr("Start")));
    m_stopButton->setText(trKey(QStringLiteral("actions.stop"), tr("Stop")));
    m_resetButton->setText(trKey(QStringLiteral("actions.reset"), tr("Reset")));
    m_devicePlaceholderText = trKey(QStringLiteral("sidebar.noDevice"), tr("No VNA Detected"));
    if (m_devicePlaceholderActive && m_deviceModel) {
        m_deviceModel->setStringList(QStringList{m_devicePlaceholderText});
        m_deviceList->setCurrentIndex(m_deviceModel->index(0));
    }
    if (m_vnaScanButton) {
        const QString scanText = trKey(QStringLiteral("sidebar.scan"), tr("Find Devices"));
        m_vnaScanButton->setToolTip(scanText);
        m_vnaScanButton->setAccessibleName(scanText);
    }
    if (m_calibrationUploadButton) {
        const QString uploadText = trKey(QStringLiteral("sidebar.upload"), tr("Upload Calibration"));
        m_calibrationUploadButton->setToolTip(uploadText);
        m_calibrationUploadButton->setAccessibleName(uploadText);
    }
}

void Sidebar::applyLocale(const QLocale& locale) {
    m_locale = locale;
    updateSpinLocale();
}

void Sidebar::buildUi() {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(16, 16, 16, 16);
    rootLayout->setSpacing(12);

    auto addSectionHeader = [this, rootLayout](const QString& key,
                                               const QString& fallback,
                                               QLabel** labelStore,
                                               QToolButton** buttonStore,
                                               const QIcon& icon,
                                               const QString& tooltip) {
        auto* container = new QWidget(this);
        container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* layout = new QHBoxLayout(container);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        auto* label = new QLabel(fallback, container);
        label->setProperty("role", "subtitle");
        label->setAccessibleDescription(fallback);
        label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        *labelStore = label;
        layout->addWidget(label);
        layout->addStretch(1);

        if (buttonStore) {
            if (!icon.isNull()) {
                auto* button = new QToolButton(container);
                button->setAutoRaise(false);
                button->setToolButtonStyle(Qt::ToolButtonIconOnly);
                button->setIcon(icon);
                button->setIconSize(QSize(20, 20));
                button->setFixedSize(32, 32);
                button->setProperty("variant", "ghost");
                const QString tip = tooltip.isEmpty() ? fallback : tooltip;
                button->setToolTip(tip);
                button->setAccessibleName(tip);
                button->setCursor(Qt::PointingHandCursor);
                layout->addWidget(button);
                *buttonStore = button;
            } else {
                *buttonStore = nullptr;
            }
        }

        rootLayout->addWidget(container);
        return label;
    };

    auto addDivider = [this, rootLayout]() {
        rootLayout->addSpacing(kSectionSpacing / 2);
        auto* divider = new QFrame(this);
        divider->setObjectName(QStringLiteral("SidebarDivider"));
        divider->setFixedHeight(1);
        divider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        rootLayout->addWidget(divider);
        rootLayout->addSpacing(kSectionSpacing / 2);
    };

    auto makeListSection = [this, rootLayout, addSectionHeader, addDivider](const QString& key,
                                                                const QString& fallback,
                                                                QListView** listStore,
                                                                QLabel** labelStore,
                                                                QToolButton** buttonStore,
                                                                const QString& objectName,
                                                                const QIcon& icon,
                                                                const QString& tooltip) {
        addSectionHeader(key, fallback, labelStore, buttonStore, icon, tooltip);

        auto* frame = new QFrame(this);
        frame->setProperty("type", "card");
        frame->setFrameShape(QFrame::NoFrame);
        frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        auto* frameLayout = new QVBoxLayout(frame);
        frameLayout->setContentsMargins(0, 0, 0, 0);

        auto* list = new QListView(frame);
        list->setObjectName(objectName);
        list->setEditTriggers(QAbstractItemView::NoEditTriggers);
        list->setSelectionMode(QAbstractItemView::SingleSelection);
        list->setUniformItemSizes(true);
        list->setMinimumHeight(120);
        list->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        if (auto* viewport = list->viewport()) {
            viewport->setAttribute(Qt::WA_AcceptTouchEvents, true);
            QScroller::grabGesture(viewport, QScroller::TouchGesture);
            QScroller::grabGesture(viewport, QScroller::LeftMouseButtonGesture);
        }
        frameLayout->addWidget(list);

        *listStore = list;
        rootLayout->addWidget(frame);
        addDivider();
    };

    const QIcon scanIcon(QStringLiteral(":/ui/theme/icons/scan.svg"));
    const QIcon uploadIcon(QStringLiteral(":/ui/theme/icons/upload.svg"));

    makeListSection(QStringLiteral("sidebar.vna"), tr("VNA"), &m_deviceList, &m_vnaLabel,
                    &m_vnaScanButton, QStringLiteral("DeviceListView"), scanIcon, tr("Find Devices"));
    makeListSection(QStringLiteral("sidebar.calibration"), tr("Calibration"), &m_calibrationList,
                    &m_calibrationLabel, &m_calibrationUploadButton, QStringLiteral("CalibrationListView"), uploadIcon,
                    tr("Upload Calibration"));

    addSectionHeader(QStringLiteral("sidebar.frequency"), tr("Frequency Range"), &m_frequencyLabel,
                     nullptr, QIcon(), QString());

    const int kControlHeight = 40;
    auto createSpinControl = [this, kControlHeight](QAbstractSpinBox* spin,
                                                    const QString& accessibleBase) -> QWidget* {
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spin->setFixedHeight(kControlHeight);
        auto* container = new QWidget(this);
        container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* layout = new QHBoxLayout(container);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(10);
        layout->addWidget(spin, 1);
        layout->setAlignment(spin, Qt::AlignVCenter);

        auto* buttonStrip = new QWidget(container);
        buttonStrip->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        auto* buttonLayout = new QHBoxLayout(buttonStrip);
        buttonLayout->setContentsMargins(0, 0, 0, 0);
        buttonLayout->setSpacing(8);
        buttonLayout->setAlignment(Qt::AlignCenter);

        auto makeButton = [&](const QString& iconPath, const QString& suffix,
                              auto stepFn) {
            auto* button = new QToolButton(buttonStrip);
            button->setIcon(QIcon(iconPath));
            button->setIconSize(QSize(16, 16));
            button->setFixedSize(kControlHeight, kControlHeight);
            button->setProperty("variant", "ghost");
            button->setAutoRepeat(true);
            button->setAutoRepeatDelay(250);
            button->setAutoRepeatInterval(60);
            button->setAccessibleName(accessibleBase + suffix);
            QObject::connect(button, &QToolButton::clicked, spin, stepFn);
            buttonLayout->addWidget(button);
            return button;
        };

        makeButton(QStringLiteral(":/ui/theme/icons/spin_up.svg"), tr(" increase"),
                   &QAbstractSpinBox::stepUp);
        makeButton(QStringLiteral(":/ui/theme/icons/spin_down.svg"), tr(" decrease"),
                   &QAbstractSpinBox::stepDown);

        layout->addWidget(buttonStrip, 0, Qt::AlignVCenter);
        return container;
    };

    auto configureSpin = [](QDoubleSpinBox* spin, double min, double max, double value) {
        spin->setDecimals(3);
        spin->setRange(min, max);
        spin->setValue(value);
        spin->setSingleStep(0.010);
        spin->setSuffix(QStringLiteral(" GHz"));
        spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };

    auto makeFrequencyField = [this, createSpinControl, configureSpin](
                                  const QString& labelKey,
                                  const QString& fallback,
                                  double min,
                                  double max,
                                  double value,
                                  QDoubleSpinBox** spinStore,
                                  QLabel** labelStore) -> QWidget* {
        auto* spin = new QDoubleSpinBox(this);
        configureSpin(spin, min, max, value);
        QWidget* control = createSpinControl(spin, fallback);

        auto* wrapper = new QWidget(this);
        wrapper->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        auto* layout = new QVBoxLayout(wrapper);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(4);

        layout->addWidget(control);

        auto* label = new QLabel(fallback, wrapper);
        label->setObjectName(labelKey);
        label->setProperty("role", "frequencyCaption");
        label->setAccessibleDescription(fallback);
        label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        layout->addWidget(label);

        *spinStore = spin;
        *labelStore = label;
        return wrapper;
    };

    m_startFrequencyField = makeFrequencyField(QStringLiteral("sidebar.startFrequency"),
                                               tr("Start Frequency"), kStartMin, kStartMax, 1.000,
                                               &m_startSpin, &m_startFrequencyLabel);
    m_endFrequencyField = makeFrequencyField(QStringLiteral("sidebar.endFrequency"),
                                             tr("End Frequency"), kEndMin, kEndMax, 6.000,
                                             &m_endSpin, &m_endFrequencyLabel);

    m_frequencyLayout = new QBoxLayout(QBoxLayout::LeftToRight);
    m_frequencyLayout->setContentsMargins(0, 0, 0, 0);
    m_frequencyLayout->setSpacing(12);
    m_frequencyLayout->addWidget(m_startFrequencyField, 1);

    m_frequencyDash = new QLabel(QStringLiteral("-"), this);
    m_frequencyDash->setAlignment(Qt::AlignCenter);
    m_frequencyDash->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_frequencyLayout->addWidget(m_frequencyDash);

    m_frequencyLayout->addWidget(m_endFrequencyField, 1);

    rootLayout->addLayout(m_frequencyLayout);
    addDivider();

    const int combinedWidth =
        m_startFrequencyField->sizeHint().width() + m_endFrequencyField->sizeHint().width() + 80;
    m_frequencyWrapThreshold = std::max(360, combinedWidth);
    updateFrequencyLayoutMode(width());

    addSectionHeader(QStringLiteral("sidebar.points"), tr("Points"), &m_pointsLabel, nullptr, QIcon(),
                     QString());

    m_pointsSpin = new QSpinBox(this);
    m_pointsSpin->setRange(1, 4096);
    m_pointsSpin->setValue(201);
    m_pointsSpin->setSingleStep(10);
    m_pointsSpin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    rootLayout->addWidget(createSpinControl(m_pointsSpin, tr("Point count")));
    addDivider();

    addSectionHeader(QStringLiteral("sidebar.parameters"), tr("S-Parameters"), &m_parametersLabel,
                     nullptr, QIcon(), QString());

    const QStringList params{QStringLiteral("S11"), QStringLiteral("S12"), QStringLiteral("S21"),
                             QStringLiteral("S22")};

    auto* paramWrapper = new QWidget(this);
    paramWrapper->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_parameterGrid = new QGridLayout(paramWrapper);
    m_parameterGrid->setContentsMargins(0, 0, 0, 0);
    m_parameterGrid->setHorizontalSpacing(8);
    m_parameterGrid->setVerticalSpacing(12);

    m_parameterRows.clear();
    m_parameterRows.reserve(params.size());

    for (const QString& param : params) {
        ParameterRow row;
        row.container = new QWidget(this);
        row.container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        auto* cellLayout = new QVBoxLayout(row.container);
        cellLayout->setContentsMargins(0, 0, 0, 0);
        cellLayout->setSpacing(6);

        auto* controlWidget = new QWidget(row.container);
        row.controlLayout = new QBoxLayout(QBoxLayout::TopToBottom);
        row.controlLayout->setContentsMargins(0, 0, 0, 0);
        row.controlLayout->setSpacing(6);
        controlWidget->setLayout(row.controlLayout);

        row.button = new QPushButton(param, controlWidget);
        row.button->setCheckable(true);
        row.button->setChecked(true);
        row.button->setMinimumHeight(m_paramButtonTallHeight);
        row.button->setMaximumHeight(m_paramButtonTallHeight);
        row.button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        row.button->setProperty("role", "param");
        row.controlLayout->addWidget(row.button);

        auto* spinWrapper = new QWidget(controlWidget);
        auto* spinLayout = new QHBoxLayout(spinWrapper);
        spinLayout->setContentsMargins(0, 0, 0, 0);
        spinLayout->setSpacing(4);

        row.spin = new QDoubleSpinBox(spinWrapper);
        row.spin->setDecimals(3);
        row.spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
        row.spin->setRange(kThresholdMin, kThresholdMax);
        row.spin->setSingleStep(0.100);
        row.spin->setSuffix(QStringLiteral(" dB"));
        row.spin->setAlignment(Qt::AlignCenter);
        row.spin->setMinimumWidth(60);
        row.spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        row.spin->setValue(-20.000);
        row.spin->setMinimumHeight(32);
        row.spin->setProperty("paramName", param);
        spinLayout->addWidget(row.spin);

        row.inputContainer = new QWidget(controlWidget);
        row.inputLayout = new QVBoxLayout(row.inputContainer);
        row.inputLayout->setContentsMargins(0, 0, 0, 0);
        row.inputLayout->setSpacing(2);
        row.inputLayout->addWidget(spinWrapper);

        row.caption = new QLabel(tr("threshold"), row.inputContainer);
        row.caption->setAlignment(Qt::AlignLeft);
        row.caption->setProperty("role", "caption");
        row.inputLayout->addWidget(row.caption);

        row.controlLayout->addWidget(row.inputContainer);
        cellLayout->addWidget(controlWidget);

        m_parameterRows.push_back(row);

        m_paramButtons.insert(param, row.button);
        m_thresholds.insert(param, row.spin);
        m_thresholdCaptions.insert(param, row.caption);
    }

    for (int c = 0; c < params.size(); ++c) {
        m_parameterGrid->setColumnStretch(c, 1);
        m_parameterGrid->addWidget(m_parameterRows[c].container, 0, c);
    }
    rootLayout->addWidget(paramWrapper);
    addDivider();

    auto* buttonRow = new QHBoxLayout;
    buttonRow->setSpacing(12);

    m_startButton = new QPushButton(tr("Start"), this);
    m_startButton->setMinimumHeight(40);
    m_startButton->setProperty("variant", "start");
    buttonRow->addWidget(m_startButton);

    m_stopButton = new QPushButton(tr("Stop"), this);
    m_stopButton->setMinimumHeight(40);
    m_stopButton->setProperty("variant", "stop");
    buttonRow->addWidget(m_stopButton);

    m_resetButton = new QPushButton(tr("Reset"), this);
    m_resetButton->setMinimumHeight(40);
    m_resetButton->setProperty("variant", "reset");
    buttonRow->addWidget(m_resetButton);

    rootLayout->addLayout(buttonRow);
    rootLayout->addStretch(1);
    updateParameterLayoutMode(width());
}

void Sidebar::bindSignals() {
    connect(m_startSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { emit rangeChanged(m_startSpin->value(), m_endSpin->value()); });
    connect(m_endSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { emit rangeChanged(m_startSpin->value(), m_endSpin->value()); });

    connect(m_pointsSpin, qOverload<int>(&QSpinBox::valueChanged), this, &Sidebar::pointsChanged);

    for (auto it = m_paramButtons.begin(); it != m_paramButtons.end(); ++it) {
        connect(it.value(), &QPushButton::toggled, this, [this, param = it.key()](bool on) {
            if (m_thresholds.contains(param)) {
                m_thresholds.value(param)->setEnabled(on);
            }
            emit parameterToggled(param, on);
        });
    }

    for (auto it = m_thresholds.begin(); it != m_thresholds.end(); ++it) {
        connect(it.value(), qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [this, param = it.key()](double value) {
                    emit thresholdChanged(param, value);
                });
    }

    connect(m_startButton, &QPushButton::clicked, this, &Sidebar::startRequested);
    connect(m_stopButton, &QPushButton::clicked, this, &Sidebar::stopRequested);
    connect(m_resetButton, &QPushButton::clicked, this, &Sidebar::resetRequested);

    if (m_vnaScanButton) {
        connect(m_vnaScanButton, &QToolButton::clicked, this, &Sidebar::vnaScanRequested);
    }
    if (m_calibrationUploadButton) {
        connect(m_calibrationUploadButton, &QToolButton::clicked, this,
                &Sidebar::calibrationUploadRequested);
    }

    if (m_deviceList) {
        if (auto* selection = m_deviceList->selectionModel()) {
            connect(selection, &QItemSelectionModel::currentChanged, this,
                    [this](const QModelIndex& current, const QModelIndex&) {
                        emit deviceSelectionChanged(current.isValid() ? current.row() : -1);
                    });
        }
        connect(m_deviceList, &QListView::doubleClicked, this, [this](const QModelIndex& index) {
            if (index.isValid()) {
                emit deviceActivated(index.row());
            }
        });
    }

    if (m_calibrationList) {
        if (auto* selection = m_calibrationList->selectionModel()) {
            connect(selection, &QItemSelectionModel::currentChanged, this,
                    [this](const QModelIndex& current, const QModelIndex&) {
                        emit calibrationSelectionChanged(current.isValid() ? current.row() : -1);
                    });
        }
        connect(m_calibrationList, &QListView::doubleClicked, this,
                [this](const QModelIndex& index) {
                    if (index.isValid()) {
                        emit calibrationActivated(index.row());
                    }
                });
    }
}

void Sidebar::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    const int newWidth = event->size().width();
    updateFrequencyLayoutMode(newWidth);
    updateParameterLayoutMode(newWidth);
}

void Sidebar::updateFrequencyLayoutMode(int availableWidth) {
    if (m_frequencyWrapThreshold <= 0 || !m_frequencyLayout || !m_frequencyDash ||
        !m_startFrequencyField || !m_endFrequencyField) {
        return;
    }

    const bool stacked = availableWidth < m_frequencyWrapThreshold;
    if (stacked == m_frequencyStacked) {
        return;
    }
    m_frequencyStacked = stacked;

    if (stacked) {
        m_frequencyLayout->setDirection(QBoxLayout::TopToBottom);
        m_frequencyLayout->setSpacing(16);
        m_frequencyDash->setVisible(false);
    } else {
        m_frequencyLayout->setDirection(QBoxLayout::LeftToRight);
        m_frequencyLayout->setSpacing(12);
        m_frequencyDash->setVisible(true);
    }
}

void Sidebar::updateParameterLayoutMode(int availableWidth) {
    if (!m_parameterGrid || m_parameterRows.empty()) {
        updateParameterButtonSizing(availableWidth);
        return;
    }

    const int threshold = m_paramStackThreshold > 0 ? m_paramStackThreshold : 520;
    const bool stacked = availableWidth < threshold;
    if (stacked != m_paramStackedLayout) {
        m_paramStackedLayout = stacked;

        for (const ParameterRow& row : m_parameterRows) {
            if (row.container) {
                m_parameterGrid->removeWidget(row.container);
            }
        }

        for (int i = 0; i < static_cast<int>(m_parameterRows.size()); ++i) {
            ParameterRow& row = m_parameterRows[static_cast<std::size_t>(i)];
            if (!row.container) {
                continue;
            }
            const int targetRow = stacked ? i : 0;
            const int targetColumn = stacked ? 0 : i;
            m_parameterGrid->addWidget(row.container, targetRow, targetColumn);
            if (row.controlLayout) {
                row.controlLayout->setDirection(stacked ? QBoxLayout::LeftToRight
                                                        : QBoxLayout::TopToBottom);
                row.controlLayout->setSpacing(stacked ? 12 : 6);
            }
        }

        if (stacked) {
            m_parameterGrid->setColumnStretch(0, 1);
            for (int c = 1; c < static_cast<int>(m_parameterRows.size()); ++c) {
                m_parameterGrid->setColumnStretch(c, 0);
            }
        } else {
            for (int c = 0; c < static_cast<int>(m_parameterRows.size()); ++c) {
                m_parameterGrid->setColumnStretch(c, 1);
            }
        }
    }

    updateParameterButtonSizing(availableWidth);
}

void Sidebar::updateParameterButtonSizing(int availableWidth) {
    if (m_paramButtons.isEmpty()) {
        return;
    }
    const int threshold = m_paramCompactThreshold > 0 ? m_paramCompactThreshold
                                                      : kParamCompactWidthThreshold;
    const bool compact = availableWidth < threshold;
    if (compact == m_paramButtonsCompact) {
        return;
    }
    m_paramButtonsCompact = compact;

    const int minHeight = compact ? m_paramButtonShortHeight : m_paramButtonTallHeight;
    const int maxHeight = compact ? m_paramButtonShortHeight : m_paramButtonTallHeight;

    for (QPushButton* button : m_paramButtons) {
        if (!button) {
            continue;
        }
        button->setMinimumHeight(minHeight);
        button->setMaximumHeight(maxHeight);
    }
}

void Sidebar::updateSpinLocale() {
    const QList<QDoubleSpinBox*> doubles{m_startSpin, m_endSpin};
    for (QDoubleSpinBox* spin : doubles) {
        spin->setLocale(m_locale);
        spin->setDecimals(3);
    }
    for (QDoubleSpinBox* spin : m_thresholds) {
        spin->setLocale(m_locale);
        spin->setDecimals(3);
        spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    }
}

void Sidebar::updateSectionTitles() {
    setLabelText(QStringLiteral("sidebar.vna"), m_vnaLabel, tr("VNA"));
    setLabelText(QStringLiteral("sidebar.calibration"), m_calibrationLabel, tr("Calibration"));
    setLabelText(QStringLiteral("sidebar.frequency"), m_frequencyLabel, tr("Frequency Range"));
    setLabelText(QStringLiteral("sidebar.startFrequency"), m_startFrequencyLabel, tr("Start Frequency"));
    setLabelText(QStringLiteral("sidebar.endFrequency"), m_endFrequencyLabel, tr("End Frequency"));
    setLabelText(QStringLiteral("sidebar.points"), m_pointsLabel, tr("Points"));
    setLabelText(QStringLiteral("sidebar.parameters"), m_parametersLabel, tr("S-Parameters"));
    if (m_startFrequencyField && m_endFrequencyField) {
        const int combinedWidth =
            m_startFrequencyField->sizeHint().width() + m_endFrequencyField->sizeHint().width() + 80;
        m_frequencyWrapThreshold = std::max(360, combinedWidth);
        updateFrequencyLayoutMode(width());
    }
}

QString Sidebar::trKey(const QString& key, const QString& fallback) const {
    return m_strings.value(key, fallback);
}

void Sidebar::setLabelText(const QString& key, QLabel* label, const QString& fallback) {
    if (!label) {
        return;
    }
    const QString text = trKey(key, fallback);
    label->setText(text);
    label->setAccessibleDescription(text);
}

} // namespace ui::views






