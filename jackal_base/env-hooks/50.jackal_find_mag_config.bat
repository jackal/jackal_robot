REM generated from jackal_base/env-hooks/50.jackal_find_mag_config.bat.in
setlocal

set _JACKAL_MAG_CONFIG=
for /f "tokens=* USEBACKQ" %%f in (`catkin_find --etc --first-only jackal_base mag_config.yaml 2^>NUL`) do (
    set _JACKAL_MAG_CONFIG=%%f
    goto :break
)
:break

if defined JACKAL_MAG_CONFIG (
    goto :eof
)

endlocal && set JACKAL_MAG_CONFIG=%_JACKAL_MAG_CONFIG%
goto :eof
