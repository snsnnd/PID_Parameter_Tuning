import logging
import sys
import threading
from logging.handlers import RotatingFileHandler
from pathlib import Path

from PySide6.QtCore import qInstallMessageHandler


def setup_logging() -> logging.Logger:
    root = logging.getLogger()
    if root.handlers:
        return logging.getLogger("ground_station")

    log_dir = Path(__file__).resolve().parents[1] / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    fmt = logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s")

    file_handler = RotatingFileHandler(log_dir / "app.log", maxBytes=2_000_000, backupCount=5, encoding="utf-8")
    file_handler.setFormatter(fmt)
    file_handler.setLevel(logging.INFO)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(fmt)
    stream_handler.setLevel(logging.WARNING)

    root.setLevel(logging.INFO)
    root.addHandler(file_handler)
    root.addHandler(stream_handler)

    install_exception_hooks()
    install_qt_message_handler()
    logging.getLogger("ground_station").info("logging initialized")
    return logging.getLogger("ground_station")


def install_exception_hooks() -> None:
    logger = logging.getLogger("ground_station.unhandled")

    def excepthook(exc_type, exc, tb):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc, tb)
            return
        logger.critical("unhandled exception", exc_info=(exc_type, exc, tb))

    def threadhook(args):
        logger.critical(
            "unhandled thread exception in %s",
            getattr(args.thread, "name", "unknown"),
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )

    sys.excepthook = excepthook
    threading.excepthook = threadhook


def install_qt_message_handler() -> None:
    logger = logging.getLogger("ground_station.qt")

    def handler(mode, context, message):
        text = f"{message} ({context.file}:{context.line})" if context.file else message
        logger.warning(text)

    qInstallMessageHandler(handler)
