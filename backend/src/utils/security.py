import html
import re
from typing import Any, Dict, List


def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent injection attacks.

    Args:
        text: Input text to sanitize

    Returns:
        Sanitized text
    """
    if not text:
        return text

    # Remove potentially harmful HTML tags
    sanitized = html.escape(text)

    # Remove potentially harmful patterns
    harmful_patterns = [
        r'<script[^>]*>.*?</script>',  # Script tags
        r'javascript:',               # JavaScript URLs
        r'vbscript:',                # VBScript URLs
        r'on\w+\s*=',                # Event handlers
        r'<iframe[^>]*>.*?</iframe>', # Iframe tags
        r'<object[^>]*>.*?</object>', # Object tags
        r'<embed[^>]*>.*?</embed>',   # Embed tags
    ]

    for pattern in harmful_patterns:
        sanitized = re.sub(pattern, '', sanitized, flags=re.IGNORECASE | re.DOTALL)

    return sanitized


def validate_api_parameters(params: Dict[str, Any], allowed_keys: List[str]) -> Dict[str, Any]:
    """
    Validate API parameters against allowed keys to prevent parameter injection.

    Args:
        params: Dictionary of parameters to validate
        allowed_keys: List of allowed parameter keys

    Returns:
        Dictionary of validated parameters
    """
    validated_params = {}

    for key, value in params.items():
        if key in allowed_keys:
            if isinstance(value, str):
                validated_params[key] = sanitize_input(value)
            else:
                validated_params[key] = value
        else:
            # Log potentially malicious parameter
            print(f"Warning: Unauthorized parameter '{key}' received and ignored")

    return validated_params


def is_safe_url(url: str) -> bool:
    """
    Check if a URL is safe to use (prevents open redirect vulnerabilities).

    Args:
        url: URL to validate

    Returns:
        True if safe, False otherwise
    """
    if not url:
        return True

    # Only allow relative URLs or URLs from trusted domains
    if url.startswith(('http://', 'https://')):
        # Add your trusted domains here
        trusted_domains = ['localhost', 'your-trusted-domain.com']
        for domain in trusted_domains:
            if domain in url:
                return True
        return False

    # Relative URLs are considered safe
    return url.startswith(('/', './', '../'))


def sanitize_query(query: str) -> str:
    """
    Sanitize search/query strings to prevent injection attacks.

    Args:
        query: Query string to sanitize

    Returns:
        Sanitized query string
    """
    if not query:
        return query

    # Remove potentially dangerous characters for search queries
    dangerous_chars = [';', '--', '/*', '*/', 'xp_', 'sp_']

    sanitized = query
    for char in dangerous_chars:
        sanitized = sanitized.replace(char, '')

    return sanitized