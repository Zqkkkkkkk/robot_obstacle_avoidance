from __future__ import annotations

import argparse
import os
from pathlib import Path

from flask import Flask, jsonify, render_template, request

from robot_obstacle_avoidance.models import obstacle_from_dict, runtime_obstacle_event_from_dict
from robot_obstacle_avoidance.scenarios import build_scenario, scenario_names
from robot_obstacle_avoidance.simulation import available_algorithms, run_all_simulations, run_simulation


def create_app() -> Flask:
    package_root = Path(__file__).resolve().parent
    app = Flask(
        __name__,
        template_folder=str(package_root / "templates"),
        static_folder=str(package_root / "static"),
    )

    @app.get("/")
    def index() -> str:
        return render_template("index.html")

    @app.get("/healthz")
    def healthz() -> object:
        return jsonify({"status": "ok"})

    @app.get("/api/meta")
    def meta() -> object:
        scenarios = [build_scenario(name).to_dict() for name in scenario_names()]
        return jsonify(
            {
                "algorithms": list(available_algorithms()),
                "scenarios": scenarios,
                "defaults": {
                    "algorithm": "astar",
                    "scenario": scenarios[0]["name"],
                    "dt": 0.1,
                },
                "obstacle_editor": {
                    "supported_types": ["circle", "rectangle"],
                    "default_radius": 0.65,
                    "default_width": 1.2,
                    "default_height": 0.8,
                },
            }
        )

    @app.route("/api/simulate", methods=["GET", "POST"])
    def simulate() -> object:
        try:
            algorithm, scenario_name, dt, custom_obstacles, runtime_obstacle_events = _parse_request_options()
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        if algorithm not in available_algorithms():
            return jsonify({"error": f"Unknown algorithm: {algorithm}"}), 400

        try:
            scenario = build_scenario(
                scenario_name,
                extra_obstacles=custom_obstacles,
                runtime_obstacle_events=runtime_obstacle_events,
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        try:
            result = run_simulation(algorithm, scenario, dt=dt)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"scenario": scenario.to_dict(), "result": result.to_dict()})

    @app.route("/api/compare", methods=["GET", "POST"])
    def compare() -> object:
        try:
            _, scenario_name, dt, custom_obstacles, runtime_obstacle_events = _parse_request_options()
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        try:
            scenario = build_scenario(
                scenario_name,
                extra_obstacles=custom_obstacles,
                runtime_obstacle_events=runtime_obstacle_events,
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        try:
            results = run_all_simulations(scenario, dt=dt)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"scenario": scenario.to_dict(), "results": [result.to_dict() for result in results]})

    return app


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the robot obstacle avoidance web visualization server.")
    parser.add_argument("--host", default=os.getenv("HOST", "127.0.0.1"))
    parser.add_argument("--port", default=int(os.getenv("PORT", "8000")), type=int)
    parser.add_argument("--debug", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    app.run(host=args.host, port=args.port, debug=args.debug)
    return 0


def _parse_dt(value: str) -> float:
    try:
        dt = float(value)
    except ValueError as exc:
        raise ValueError("dt must be a floating-point number") from exc

    if dt <= 0.0:
        raise ValueError("dt must be greater than zero")
    return dt


def _parse_request_options() -> tuple[str, str, float, tuple[object, ...], tuple[object, ...]]:
    if request.method == "POST":
        payload = request.get_json(silent=True) or {}
        algorithm = str(payload.get("algorithm", "astar"))
        scenario_name = str(payload.get("scenario", "open_field"))
        dt = _parse_dt(str(payload.get("dt", "0.1")))
        custom_obstacles = _parse_custom_obstacles(payload.get("custom_obstacles", []))
        runtime_obstacle_events = _parse_runtime_obstacle_events(payload.get("runtime_obstacle_events", []))
        return algorithm, scenario_name, dt, custom_obstacles, runtime_obstacle_events

    return (
        request.args.get("algorithm", "astar"),
        request.args.get("scenario", "open_field"),
        _parse_dt(request.args.get("dt", "0.1")),
        tuple(),
        tuple(),
    )


def _parse_custom_obstacles(raw_value: object) -> tuple[object, ...]:
    if raw_value in (None, []):
        return tuple()
    if not isinstance(raw_value, list):
        raise ValueError("custom_obstacles must be a list")

    parsed = []
    for item in raw_value:
        if not isinstance(item, dict):
            raise ValueError("Each custom obstacle must be an object")
        parsed.append(obstacle_from_dict(item))
    return tuple(parsed)


def _parse_runtime_obstacle_events(raw_value: object) -> tuple[object, ...]:
    if raw_value in (None, []):
        return tuple()
    if not isinstance(raw_value, list):
        raise ValueError("runtime_obstacle_events must be a list")

    parsed = []
    for item in raw_value:
        if not isinstance(item, dict):
            raise ValueError("Each runtime obstacle event must be an object")
        parsed.append(runtime_obstacle_event_from_dict(item))
    return tuple(parsed)


app = create_app()


if __name__ == "__main__":
    raise SystemExit(main())
