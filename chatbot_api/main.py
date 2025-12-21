

@app.get("/debug-secrets-temp")
def debug_secrets():
    # WARNING: This is for debugging only and exposes secrets.
    # This endpoint should be removed after the issue is resolved.
    return {
        "NEON_DB_URL_VALUE": settings.NEON_DB_URL,
        "GEMINI_API_KEY_LOADED": "Yes" if settings.GEMINI_API_KEY else "No",
        "QDRANT_URL_LOADED": "Yes" if settings.QDRANT_URL else "No",
        "QDRANT_API_KEY_LOADED": "Yes" if settings.QDRANT_API_KEY else "No",
    }
