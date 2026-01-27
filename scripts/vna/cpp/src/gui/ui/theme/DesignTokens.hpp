// Copyright (c) 2025 vna-spoofer
// SPDX-License-Identifier: MIT

#pragma once

#include <QString>

namespace ui::theme::Tokens {

// Colors
inline constexpr auto BG_Canvas = "#E0D8D1";
inline constexpr auto BG_Card = "#D5CCC4";
inline constexpr auto InkPrimary = "#221F1B";
inline constexpr auto InkMuted = "#6B655F";
inline constexpr auto StrokeSoft = "#CABEB4";
inline constexpr auto StrokeGrid = "#D7CCC2";
inline constexpr auto Focus = "#5A4FCF";
inline constexpr auto AccentParam = "#635C56";
inline constexpr auto AccentStart = "#423C38";
inline constexpr auto AccentStop = "#B95C5C";
inline constexpr auto AccentResetBorder = "#635C56";
inline constexpr auto InkOnAccent = "#E0D8D1";
inline constexpr auto ResultPassBg = "#C8F7A6";
inline constexpr auto ResultPassBorder = "#7DD45A";
inline constexpr auto ResultPassText = "#1F4F1C";
inline constexpr auto ResultFailBg = "#F9B0B0";
inline constexpr auto ResultFailBorder = "#E26C6C";
inline constexpr auto ResultFailText = "#611B1B";
inline constexpr auto ResultDisabledBg = "#E2DFDC";
inline constexpr auto ResultDisabledBorder = "#CAC2BC";
inline constexpr auto ResultDisabledText = "#6B655F";
inline constexpr auto ResultPendingBg = "#DED6CD";
inline constexpr auto ResultPendingBorder = "#CABEB4";
inline constexpr auto ResultPendingText = "#6B655F";

// Series colors
inline constexpr auto S11 = "#E2554E";
inline constexpr auto S12 = "#3A72E6";
inline constexpr auto S21 = "#E5C148";
inline constexpr auto S22 = "#6E5CE6";

enum class Radius {
    XS = 4,
    SM = 6,
    MD = 8,
    LG = 12
};

enum class Space {
    S0 = 0,
    S1 = 4,
    S2 = 8,
    S3 = 12,
    S4 = 16,
    S5 = 24,
    S6 = 32
};

namespace Font {
inline constexpr int XS = 10;
inline constexpr int SM = 11;
inline constexpr int MD = 12;
inline constexpr int LG = 14;
inline constexpr auto Family =
    "Anonymous Pro, JetBrains Mono, IBM Plex Mono, Consolas, 'Courier New', monospace";
} // namespace Font

inline int space(Space value) {
    return static_cast<int>(value);
}

inline int radius(Radius value) {
    return static_cast<int>(value);
}

inline QString fontFamily() {
    return QString::fromUtf8(Font::Family);
}

} // namespace ui::theme::Tokens
