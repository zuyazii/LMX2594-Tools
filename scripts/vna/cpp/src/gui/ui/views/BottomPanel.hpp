// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include "../widgets/SeriesCheck.hpp"

#include <QHash>
#include <QLocale>
#include <QWidget>
#include <QStringList>

class QComboBox;
class QLabel;
class QCheckBox;

namespace ui::views {

class BottomPanel : public QWidget {
    Q_OBJECT

public:
    explicit BottomPanel(QWidget* parent = nullptr);

    void setSeriesEnabled(const QHash<QString, bool>& enabled);
    void setMagnitudeState(const QHash<QString, bool>& state);
    void setPhaseState(const QHash<QString, bool>& state);
    void setSeriesColors(const QHash<QString, QColor>& magnitudeColors,
                         const QHash<QString, QColor>& phaseColors);
    void setMagnitudeVisible(bool on);
    void setPhaseVisible(bool on);
    enum class ResultState {
        Pending,
        Passed,
        Failed,
        Disabled,
        NoData
    };
    void setResultState(const QString& name, ResultState state);
    void setResultStates(const QHash<QString, ResultState>& states);
    void resetResultStates(const QStringList& activeParameters);

    void applyTranslations(const QHash<QString, QString>& strings);
    void applyLocale(const QLocale& locale);

signals:
    void languageChanged(const QLocale& locale);
    void magnitudeColorChanged(const QString& name, const QColor& color);
    void phaseColorChanged(const QString& name, const QColor& color);
    void magnitudeAllToggled(bool on);
    void phaseAllToggled(bool on);
    void seriesMagnitudeToggled(const QString& name, bool on);
    void seriesPhaseToggled(const QString& name, bool on);

private:
    void buildUi();
    void bindSignals();
    void updateTexts();
    QString trKey(const QString& key, const QString& fallback) const;
    void updatePhaseColor(const QString& name, const QColor& color);
    void setLanguageSelection(const QString& key);
    void syncMagnitudeHeader();
    void syncPhaseHeader();
    void applyResultState(const QString& name, ResultState state);
    QString resultText(ResultState state) const;
    QString resultStateProperty(ResultState state) const;
    void refreshResultTexts();

    QLabel* m_languageLabel = nullptr;
    QLabel* m_resultsLabel = nullptr;
    QHash<QString, QCheckBox*> m_languageChecks;
    QCheckBox* m_magnitudeHeader = nullptr;
    QCheckBox* m_phaseHeader = nullptr;
    QHash<QString, widgets::SeriesCheck*> m_magnitudeChecks;
    QHash<QString, widgets::SeriesCheck*> m_phaseChecks;
    QHash<QString, QLabel*> m_resultLabels;
    QHash<QString, ResultState> m_resultStates;

    QHash<QString, QString> m_strings;
    QLocale m_locale;
    QString m_activeLanguageKey;
    QHash<QString, bool> m_enabledSeries;
    bool m_magnitudeVisibleGlobal = true;
    bool m_phaseVisibleGlobal = true;
    bool m_blockLanguage = false;
    bool m_blockMagnitude = false;
    bool m_blockPhase = false;
};

} // namespace ui::views
