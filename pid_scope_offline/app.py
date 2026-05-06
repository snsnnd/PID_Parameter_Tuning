from PySide6.QtWidgets import QApplication
from ui.main_window import MainWindow
import sys


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
