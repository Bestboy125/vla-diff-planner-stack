FROM ground-image:latest

WORKDIR /opt/vla-ground-station

COPY linux/requirements-linux.txt ./requirements-linux.txt
RUN python3 -m pip install --no-cache-dir -r requirements-linux.txt

COPY backend/app ./ground_station/backend/app
COPY frontend/dist ./ground_station/frontend/dist

WORKDIR /opt/vla-ground-station/ground_station/backend
EXPOSE 8080

HEALTHCHECK --interval=15s --timeout=3s --start-period=10s --retries=3 \
  CMD python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8080/api/missions/current', timeout=2)" || exit 1

ENTRYPOINT []
CMD ["python3", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080"]
