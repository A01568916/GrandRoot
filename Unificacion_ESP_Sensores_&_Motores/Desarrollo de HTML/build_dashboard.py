#!/usr/bin/env python3
"""
build_dashboard.py — Genera grandroot_dashboard_offline.html e index_html.h
inyectando Leaflet y Chart.js en el HTML base.

Uso:
    python3 build_dashboard.py
"""

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def leer(nombre):
    with open(os.path.join(BASE_DIR, nombre), 'r', encoding='utf-8') as f:
        return f.read()

def escribir(nombre, contenido):
    with open(os.path.join(BASE_DIR, nombre), 'w', encoding='utf-8') as f:
        f.write(contenido)

def main():
    print("=" * 60)
    print("  GrandRoot — Build Dashboard")
    print("=" * 60)

    # Leer todos los pedazos
    html        = leer('grandroot_dashboard.html')
    leaflet_css = leer('leaflet.css')
    leaflet_js  = leer('leaflet.js')
    chart_js    = leer('chart.js')

    print(f"\n[INPUT] grandroot_dashboard.html : {len(html):>8} bytes")
    print(f"[INPUT] leaflet.css              : {len(leaflet_css):>8} bytes")
    print(f"[INPUT] leaflet.js               : {len(leaflet_js):>8} bytes")
    print(f"[INPUT] chart.js                 : {len(chart_js):>8} bytes")

    # Sanitizar JavaScript para inyección segura en <script>
    # NUNCA debe aparecer </script en el contenido. Reemplazar por <\/script.
    def sanitize_js(js):
        return js.replace('</script', '<\\/script')

    leaflet_js = sanitize_js(leaflet_js)
    chart_js   = sanitize_js(chart_js)

    # El CSS de Leaflet referencia url(images/...) que no tenemos.
    # Esos 404s son inofensivos (solo afectan controles que no usamos),
    # pero los reemplazamos por gif transparente embebido para evitar
    # spam en la consola.
    TRANSPARENT_GIF = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw=='
    leaflet_css = leaflet_css.replace(
        'url(images/marker-icon.png)', f'url({TRANSPARENT_GIF})'
    ).replace(
        'url(images/layers-2x.png)', f'url({TRANSPARENT_GIF})'
    ).replace(
        'url(images/layers.png)', f'url({TRANSPARENT_GIF})'
    )

    # Inyectar
    out = html.replace('/*__LEAFLET_CSS__*/', leaflet_css)
    out = out.replace('/*__LEAFLET_JS__*/',   leaflet_js)
    out = out.replace('/*__CHART_JS__*/',     chart_js)

    # Sanity: verificar que no quedan placeholders sin reemplazar
    for ph in ['__LEAFLET_CSS__', '__LEAFLET_JS__', '__CHART_JS__']:
        if ph in out:
            print(f"\n[ERROR] Placeholder no reemplazado: {ph}")
            sys.exit(1)

    print(f"\n[OUTPUT] HTML embebido        : {len(out):>8} bytes (~{len(out)//1024} KB)")

    # 1) Versión standalone para probar en PC
    escribir('grandroot_dashboard_offline.html', out)
    print(f"[OUTPUT] grandroot_dashboard_offline.html  ✓")

    # 2) Versión para el ESP32 (index_html.h)
    # Verificar que el HTML no contenga )rawliteral" (no la contiene en la práctica
    # pero hay que ser paranoicos). Si la tuviera, cambiamos el delimitador.
    delim = 'GRANDROOT'
    if f'){delim}"' in out:
        print(f"[ERROR] El HTML contiene ){delim}\" — cambiando delimitador")
        delim = 'GRANDROOT_X'

    header = (
        '// index_html.h\n'
        '// Dashboard GrandRoot unificado — TODO embebido (Leaflet + Chart.js inline).\n'
        '// Funciona 100% offline conectado al AP del ESP32.\n'
        '// Generado automáticamente por build_dashboard.py.\n'
        '\n'
        '#pragma once\n'
        '\n'
        f'const char INDEX_HTML[] PROGMEM = R"{delim}(\n'
        + out +
        f'\n){delim}";\n'
    )

    escribir('index_html.h', header)
    print(f"[OUTPUT] index_html.h         : {len(header):>8} bytes (~{len(header)//1024} KB)")

    # Aviso de tamaño
    kb = len(header) // 1024
    print()
    if kb > 800:
        print(f"[AVISO] El .h pesa {kb} KB — verifica que cabe en PROGMEM.")
        print(f"        ESP32 típico tiene ~1300 KB libres tras WiFi/WebServer.")
    else:
        print(f"[OK] El .h pesa {kb} KB — entra sin problema en PROGMEM.")

    print("\n¡Listo!")

if __name__ == '__main__':
    main()
