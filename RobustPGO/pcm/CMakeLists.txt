# Install headers
file(GLOB headers_pcm "*.h")
install(FILES ${headers_pcm} DESTINATION include/RobustPGO/pcm)

