FROM python:3.9-slim-bullseye

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# Runs as root on purpose: settings are stored in /root/.config, which BlueOS bind-mounts from the host
# (see "Binds" below), so they survive container restarts and updates.
COPY nucleus_extension /app/nucleus_extension
WORKDIR /app

ENV PYTHONUNBUFFERED=1 PORT=8000
EXPOSE 8000/tcp

LABEL version="0.1.0"
LABEL permissions='\
{\
  "ExposedPorts": {\
    "8000/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": ["/root/.config:/root/.config"],\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "8000/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors='[\
    {\
        "name": "LukeGrif",\
        "email": ""\
    }\
]'
LABEL company='{\
        "about": "Unofficial Nucleus DVL extension with a configurable mounting rotation, based on the original Nortek extension",\
        "name": "",\
        "email": ""\
    }'
LABEL type="device-integration"
LABEL tags='[\
    "positioning",\
    "navigation",\
    "doppler-velocity-log"\
]'
LABEL readme='https://raw.githubusercontent.com/LukeGrif/nucleus_driver/main/blueos_extension/EXTENSION.md'
LABEL links='{\
        "website": "https://github.com/LukeGrif/nucleus_driver/tree/main/blueos_extension",\
        "support": "https://github.com/LukeGrif/nucleus_driver/issues"\
    }'
LABEL requirements="core >= 1.1"

CMD ["python", "-m", "nucleus_extension.main"]
