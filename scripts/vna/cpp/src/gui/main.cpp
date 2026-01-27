#include "ui/theme/ThemeLoader.hpp"
#include "ui/views/MainView.hpp"

#include <QApplication>
#include <QGuiApplication>
#include <QMainWindow>

int main(int argc, char* argv[]) {
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("VNA Spoofer"));
    QApplication::setOrganizationName(QStringLiteral("vna-spoofer"));

    ui::theme::ThemeLoader::apply(app);

    auto* window = new QMainWindow;
    window->setObjectName(QStringLiteral("AppWindow"));
    auto* mainView = new ui::views::MainView(window);
    window->setCentralWidget(mainView);
    ui::theme::ThemeLoader::applyWindowChrome(window);
    window->resize(1280, 800);
    window->show();

    return app.exec();
}
