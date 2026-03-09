# Deployment

## Render

This repository is ready for Render deployment.

1. Push the `robot_obstacle_avoidance` repository to GitHub.
2. In Render, choose `New +` -> `Blueprint` or `Web Service`.
3. Point Render to this repository.
4. Render will detect `render.yaml` automatically.
5. After the first deploy, open `/healthz` to verify the service is healthy.

Expected public paths:

- `/`: main dashboard
- `/healthz`: health check endpoint
- `/api/meta`: scenario and algorithm metadata
- `/api/simulate`: single-algorithm simulation API
- `/api/compare`: compare-all simulation API

## Docker

You can also deploy the included Docker image to any container platform.

### Build locally

```bash
docker build -t robot-obstacle-avoidance .
```

### Run locally

```bash
docker run --rm -p 8000:8000 -e PORT=8000 robot-obstacle-avoidance
```

Then open `http://127.0.0.1:8000`.

## Notes

- The service uses `waitress` instead of Flask's development server.
- The deployment assumes the repository root is `robot_obstacle_avoidance/`.
- Runtime obstacle insertion and compare-all mode are supported in deployed environments.
