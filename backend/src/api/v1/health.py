from fastapi import APIRouter
from datetime import datetime
from typing import Dict, Any

from src.config.settings import settings


router = APIRouter(prefix=settings.api_v1_prefix, tags=["health"])


@router.get("/health")
async def health_check() -> Dict[str, Any]:
    """
    Check the health status of the RAG service
    """
    # In a real implementation, you would check actual connections to services
    # For now, we'll simulate the check

    # Simulate checking services
    qdrant_status = "connected"  # In real implementation, would check actual connection
    neon_status = "connected"   # In real implementation, would check actual connection
    openai_router_status = "available"  # In real implementation, would check actual connection

    overall_status = "healthy"

    # If any service is not available, mark as degraded or unhealthy
    if qdrant_status != "connected" or neon_status != "connected" or openai_router_status != "available":
        overall_status = "degraded"

    return {
        "status": overall_status,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "services": {
            "qdrant": qdrant_status,
            "neon": neon_status,
            "openai_router": openai_router_status
        },
        "version": "1.0.0"
    }