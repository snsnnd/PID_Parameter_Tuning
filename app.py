from PySide6.QtWidgets import QApplication
from core.logging_setup import setup_logging
from ui.main_window import MainWindow
import sys


def main() -> int:
    setup_logging()
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
