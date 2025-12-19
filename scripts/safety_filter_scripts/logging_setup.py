import sys
import logging

def setup_logger(
        name: str = 'SF',
        logging_file: str | None = None,
        file_only: bool = False
) -> logging.Logger:
    logger = logging.getLogger(name)
    log_formatter = logging.Formatter("[%(asctime)s] [%(name)s] [%(levelname)s]  %(message)s")

    if logging_file is not None:
        file_handler = logging.FileHandler(logging_file)
        file_handler.setFormatter(log_formatter)
        logger.addHandler(file_handler)
    
    if not file_only or logging_file is None:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(log_formatter)
        logger.addHandler(console_handler)
    return logger