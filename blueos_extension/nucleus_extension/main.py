"""Entry point: load settings, start the Nucleus link thread and serve the web page / REST API."""

import logging
import os

from nucleus_extension.api import create_app
from nucleus_extension.link import NucleusLink
from nucleus_extension.settings import load_settings

PORT = int(os.environ.get("PORT", "8000"))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)  # the page polls /api/status every second
    link = NucleusLink(load_settings())
    link.start_thread()
    create_app(link).run(host="0.0.0.0", port=PORT, threaded=True)


if __name__ == "__main__":
    main()
