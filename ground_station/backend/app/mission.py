import asyncio
from uuid import UUID

from fastapi import HTTPException, status

from .schemas import Mission, MissionCreate, MissionMode, MissionState, utc_now


TERMINAL_STATES = {MissionState.SUCCEEDED, MissionState.ABORTED, MissionState.FAULT}


class MissionManager:
    def __init__(self, control_output_enabled: bool = False) -> None:
        self._lock = asyncio.Lock()
        self._mission: Mission | None = None
        self.control_output_enabled = control_output_enabled

    async def current(self) -> Mission | None:
        async with self._lock:
            return self._mission.model_copy(deep=True) if self._mission else None

    async def create(self, request: MissionCreate) -> Mission:
        async with self._lock:
            if self._mission and self._mission.state not in TERMINAL_STATES:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail="An active mission already exists; stop it before creating another.",
                )
            self._mission = Mission(**request.model_dump())
            return self._mission.model_copy(deep=True)

    async def start(self, mission_id: UUID) -> Mission:
        async with self._lock:
            mission = self._require(mission_id)
            if mission.mode == MissionMode.LIVE and not self.control_output_enabled:
                raise HTTPException(
                    status_code=status.HTTP_423_LOCKED,
                    detail="Live control is safety-locked until the onboard bridge and watchdog pass validation.",
                )
            if mission.state not in {MissionState.ARMED, MissionState.HOLDING}:
                raise HTTPException(status_code=409, detail=f"Cannot start mission from {mission.state}.")
            mission.state = MissionState.RUNNING
            mission.status_message = "Dry-run mission active." if mission.mode == MissionMode.DRY_RUN else "Live mission active."
            mission.updated_at = utc_now()
            return mission.model_copy(deep=True)

    async def hold(self, mission_id: UUID) -> Mission:
        async with self._lock:
            mission = self._require(mission_id)
            if mission.state != MissionState.RUNNING:
                raise HTTPException(status_code=409, detail=f"Cannot hold mission from {mission.state}.")
            mission.state = MissionState.HOLDING
            mission.status_message = "Mission is holding; no new motion intent will be emitted."
            mission.updated_at = utc_now()
            return mission.model_copy(deep=True)

    async def stop(self, mission_id: UUID) -> Mission:
        async with self._lock:
            mission = self._require(mission_id)
            if mission.state in TERMINAL_STATES:
                return mission.model_copy(deep=True)
            mission.state = MissionState.ABORTED
            mission.status_message = "Mission stopped by operator."
            mission.updated_at = utc_now()
            return mission.model_copy(deep=True)

    def _require(self, mission_id: UUID) -> Mission:
        if not self._mission or self._mission.mission_id != mission_id:
            raise HTTPException(status_code=404, detail="Mission not found.")
        return self._mission
