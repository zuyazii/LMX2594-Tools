#include "ThemeLoader.hpp"

#include "DesignTokens.hpp"

#include <QColor>
#include <QFile>
#include <QFont>
#include <QFontDatabase>
#include <QMap>
#include <QTextStream>
#include <QStringConverter>
#include <QStringList>
#include <QWidget>
#include <algorithm>

#if defined(Q_OS_WIN)
#define NOMINMAX
#include <windows.h>
#include <QLibrary>
#include <QOperatingSystemVersion>
#endif

namespace ui::theme {

namespace {

QString readResourceFile(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    return stream.readAll();
}

QColor adjustLuminance(const QColor& color, double factor) {
    QColor out = color;
    int r = color.red();
    int g = color.green();
    int b = color.blue();

    auto mix = [factor](int channel) -> int {
        const double value = channel * factor;
        return std::clamp(static_cast<int>(value), 0, 255);
    };

    out.setRed(mix(r));
    out.setGreen(mix(g));
    out.setBlue(mix(b));
    return out;
}

QString hex(const QColor& color) {
    return color.name(QColor::HexRgb);
}

} // namespace

void ThemeLoader::apply(QApplication& app) {
    ensureFontsRegistered();
    QFont appFont(QStringLiteral("Anonymous Pro"));
    if (appFont.family().compare(QStringLiteral("Anonymous Pro"), Qt::CaseInsensitive) != 0) {
        appFont = app.font();
        appFont.setFamily(QString::fromUtf8(Tokens::Font::Family).split(',').first().trimmed());
    }
    appFont.setPointSize(Tokens::Font::MD);
    app.setFont(appFont);

    app.setStyleSheet(stylesheet());
}

QString ThemeLoader::stylesheet() {
    static QString cached;
    if (cached.isEmpty()) {
        const QString tmpl = loadTemplate();
        cached = applyTokens(tmpl);
    }
    return cached;
}

void ThemeLoader::applyWindowChrome(QWidget* widget) {
#if defined(Q_OS_WIN)
    if (!widget) {
        return;
    }

    QWidget* topLevel = widget->window();
    if (!topLevel) {
        return;
    }

    topLevel->winId();
    const HWND hwnd = reinterpret_cast<HWND>(topLevel->winId());
    if (!hwnd) {
        return;
    }

    if (QOperatingSystemVersion::current() < QOperatingSystemVersion::Windows11) {
        return;
    }

    static QLibrary dwmLib(QStringLiteral("dwmapi"));
    if (!dwmLib.isLoaded() && !dwmLib.load()) {
        return;
    }

    using DwmSetWindowAttributeFn = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
    auto* setAttribute =
        reinterpret_cast<DwmSetWindowAttributeFn>(dwmLib.resolve("DwmSetWindowAttribute"));
    if (!setAttribute) {
        return;
    }

    constexpr DWORD kBorderAttr = 34;  // DWMWA_BORDER_COLOR
    constexpr DWORD kCaptionAttr = 35; // DWMWA_CAPTION_COLOR
    constexpr DWORD kTextAttr = 36;    // DWMWA_TEXT_COLOR
    constexpr DWORD kHeightAttr = 37;  // DWMWA_CAPTION_HEIGHT

    const QColor captionColor(QString::fromUtf8(Tokens::BG_Card));
    const COLORREF captionRef = RGB(captionColor.red(), captionColor.green(), captionColor.blue());
    setAttribute(hwnd, kCaptionAttr, &captionRef, sizeof(captionRef));

    const QColor textColor(QString::fromUtf8(Tokens::InkPrimary));
    const COLORREF textRef = RGB(textColor.red(), textColor.green(), textColor.blue());
    setAttribute(hwnd, kTextAttr, &textRef, sizeof(textRef));

    const QColor borderColor(QString::fromUtf8(Tokens::InkPrimary));
    const COLORREF borderRef = RGB(borderColor.red(), borderColor.green(), borderColor.blue());
    setAttribute(hwnd, kBorderAttr, &borderRef, sizeof(borderRef));

    const int captionHeight = 32; // compact title bar to match the in-app chrome
    setAttribute(hwnd, kHeightAttr, &captionHeight, sizeof(captionHeight));
#else
    (void)widget;
#endif
}

void ThemeLoader::ensureFontsRegistered() {
    static bool registered = false;
    if (registered) {
        return;
    }

    const QString fontPath = QStringLiteral(":/ui/theme/fonts/AnonymousPro-Regular.ttf");
    if (QFontDatabase::addApplicationFont(fontPath) == -1) {
        // Continue even if the font fails to load; Qt will fall back automatically.
    }
    registered = true;
}

QString ThemeLoader::loadTemplate() {
    return readResourceFile(QStringLiteral(":/ui/theme/theme.qss"));
}

QString ThemeLoader::applyTokens(const QString& tmpl) {
    using namespace Tokens;

    const QColor bgCard(QString::fromUtf8(BG_Card));
    const QColor hover = adjustLuminance(bgCard, 0.96); // ~4% darker
    const QColor accentParam(QString::fromUtf8(AccentParam));
    const QColor accentStart(QString::fromUtf8(AccentStart));
    const QColor accentStop(QString::fromUtf8(AccentStop));

    const QColor accentParamHover = adjustLuminance(accentParam, 1.08);
    const QColor accentParamPressed = adjustLuminance(accentParam, 0.88);
    const QColor accentStartHover = adjustLuminance(accentStart, 1.08);
    const QColor accentStartPressed = adjustLuminance(accentStart, 0.88);
    const QColor accentStopHover = adjustLuminance(accentStop, 1.08);
    const QColor accentStopPressed = adjustLuminance(accentStop, 0.88);

    const QMap<QString, QString> replacements{
        {QStringLiteral("{{FontFamily}}"), QString::fromUtf8(Font::Family)},
        {QStringLiteral("{{InkPrimary}}"), QString::fromUtf8(InkPrimary)},
        {QStringLiteral("{{InkMuted}}"), QString::fromUtf8(InkMuted)},
        {QStringLiteral("{{BG_Canvas}}"), QString::fromUtf8(BG_Canvas)},
        {QStringLiteral("{{BG_Card}}"), QString::fromUtf8(BG_Card)},
        {QStringLiteral("{{StrokeSoft}}"), QString::fromUtf8(StrokeSoft)},
        {QStringLiteral("{{StrokeGrid}}"), QString::fromUtf8(StrokeGrid)},
        {QStringLiteral("{{Focus}}"), QString::fromUtf8(Focus)},
        {QStringLiteral("{{Hover}}"), hex(hover)},
        {QStringLiteral("{{AccentParam}}"), QString::fromUtf8(AccentParam)},
        {QStringLiteral("{{AccentParamHover}}"), hex(accentParamHover)},
        {QStringLiteral("{{AccentParamPressed}}"), hex(accentParamPressed)},
        {QStringLiteral("{{AccentStart}}"), QString::fromUtf8(AccentStart)},
        {QStringLiteral("{{AccentStartHover}}"), hex(accentStartHover)},
        {QStringLiteral("{{AccentStartPressed}}"), hex(accentStartPressed)},
        {QStringLiteral("{{AccentStop}}"), QString::fromUtf8(AccentStop)},
        {QStringLiteral("{{AccentStopHover}}"), hex(accentStopHover)},
        {QStringLiteral("{{AccentStopPressed}}"), hex(accentStopPressed)},
        {QStringLiteral("{{InkOnAccent}}"), QString::fromUtf8(InkOnAccent)},
        {QStringLiteral("{{AccentResetBorder}}"), QString::fromUtf8(AccentResetBorder)},
        {QStringLiteral("{{RadiusXS}}"), QString::number(radius(Radius::XS))},
        {QStringLiteral("{{RadiusSM}}"), QString::number(radius(Radius::SM))},
        {QStringLiteral("{{RadiusMD}}"), QString::number(radius(Radius::MD))},
        {QStringLiteral("{{RadiusLG}}"), QString::number(radius(Radius::LG))},
        {QStringLiteral("{{ResultPassBg}}"), QString::fromUtf8(ResultPassBg)},
        {QStringLiteral("{{ResultPassBorder}}"), QString::fromUtf8(ResultPassBorder)},
        {QStringLiteral("{{ResultPassText}}"), QString::fromUtf8(ResultPassText)},
        {QStringLiteral("{{ResultFailBg}}"), QString::fromUtf8(ResultFailBg)},
        {QStringLiteral("{{ResultFailBorder}}"), QString::fromUtf8(ResultFailBorder)},
        {QStringLiteral("{{ResultFailText}}"), QString::fromUtf8(ResultFailText)},
        {QStringLiteral("{{ResultDisabledBg}}"), QString::fromUtf8(ResultDisabledBg)},
        {QStringLiteral("{{ResultDisabledBorder}}"), QString::fromUtf8(ResultDisabledBorder)},
        {QStringLiteral("{{ResultDisabledText}}"), QString::fromUtf8(ResultDisabledText)},
        {QStringLiteral("{{ResultPendingBg}}"), QString::fromUtf8(ResultPendingBg)},
        {QStringLiteral("{{ResultPendingBorder}}"), QString::fromUtf8(ResultPendingBorder)},
        {QStringLiteral("{{ResultPendingText}}"), QString::fromUtf8(ResultPendingText)}
    };

    QString out = tmpl;
    for (auto it = replacements.cbegin(); it != replacements.cend(); ++it) {
        out.replace(it.key(), it.value());
    }
    return out;
}

} // namespace ui::theme
