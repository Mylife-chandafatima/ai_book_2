import time
import logging
from datetime import datetime
from typing import Dict, Any, Optional
from dataclasses import dataclass


logger = logging.getLogger(__name__)


@dataclass
class QueryMetrics:
    """Data class to hold query performance metrics"""
    query_id: str
    start_time: float
    end_time: Optional[float] = None
    processing_time_ms: Optional[float] = None
    success: bool = True
    error_message: Optional[str] = None
    user_id: Optional[str] = None
    session_id: Optional[str] = None


class MonitoringService:
    """Service for monitoring performance and collecting metrics"""

    def __init__(self):
        self.metrics: Dict[str, QueryMetrics] = {}
        self.performance_threshold = 5000  # 5 seconds in ms

    def start_query_monitoring(self, query_id: str, user_id: Optional[str] = None, session_id: Optional[str] = None) -> str:
        """
        Start monitoring a query.

        Args:
            query_id: Unique identifier for the query
            user_id: Optional user identifier
            session_id: Optional session identifier

        Returns:
            Query ID for tracking
        """
        self.metrics[query_id] = QueryMetrics(
            query_id=query_id,
            start_time=time.time(),
            user_id=user_id,
            session_id=session_id
        )
        return query_id

    def end_query_monitoring(self, query_id: str, success: bool = True, error_message: Optional[str] = None) -> Optional[float]:
        """
        End monitoring for a query and calculate performance metrics.

        Args:
            query_id: Query ID to end monitoring for
            success: Whether the query was successful
            error_message: Optional error message if unsuccessful

        Returns:
            Processing time in milliseconds, or None if query not found
        """
        if query_id not in self.metrics:
            logger.warning(f"Query {query_id} not found in metrics")
            return None

        metric = self.metrics[query_id]
        metric.end_time = time.time()
        metric.processing_time_ms = (metric.end_time - metric.start_time) * 1000
        metric.success = success
        metric.error_message = error_message

        # Log performance if it exceeds threshold
        if metric.processing_time_ms > self.performance_threshold:
            logger.warning(f"Slow query detected: {query_id} took {metric.processing_time_ms:.2f}ms")

        # Log the metric
        self._log_metric(metric)

        # Remove from active metrics to prevent memory buildup
        processing_time = metric.processing_time_ms
        del self.metrics[query_id]

        return processing_time

    def _log_metric(self, metric: QueryMetrics):
        """Log the metric to the application logs."""
        status = "SUCCESS" if metric.success else "ERROR"
        logger.info(
            f"Query {metric.query_id}: {status} | "
            f"Time: {metric.processing_time_ms:.2f}ms | "
            f"User: {metric.user_id} | "
            f"Session: {metric.session_id} | "
            f"Error: {metric.error_message or 'None'}"
        )

    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get overall performance statistics.

        Returns:
            Dictionary with performance statistics
        """
        if not self.metrics:
            return {
                "total_queries": 0,
                "avg_response_time_ms": 0,
                "max_response_time_ms": 0,
                "min_response_time_ms": 0,
                "slow_queries": 0
            }

        processing_times = [m.processing_time_ms for m in self.metrics.values() if m.processing_time_ms is not None]

        if not processing_times:
            return {
                "total_queries": len(self.metrics),
                "avg_response_time_ms": 0,
                "max_response_time_ms": 0,
                "min_response_time_ms": 0,
                "slow_queries": 0
            }

        avg_time = sum(processing_times) / len(processing_times)
        max_time = max(processing_times)
        min_time = min(processing_times)
        slow_queries = sum(1 for t in processing_times if t > self.performance_threshold)

        return {
            "total_queries": len(self.metrics),
            "avg_response_time_ms": avg_time,
            "max_response_time_ms": max_time,
            "min_response_time_ms": min_time,
            "slow_queries": slow_queries
        }

    def add_custom_metric(self, name: str, value: Any, tags: Optional[Dict[str, str]] = None):
        """
        Add a custom metric for monitoring.

        Args:
            name: Name of the metric
            value: Value of the metric
            tags: Optional tags to associate with the metric
        """
        logger.info(f"Custom Metric - {name}: {value} | Tags: {tags or {}}")


# Global monitoring service instance
monitoring_service = MonitoringService()