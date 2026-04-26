#!/usr/bin/env bash
# setup_geographiclib.sh — installe le dataset EGM96 requis par MAVROS
# À exécuter une seule fois sur le Raspberry Pi.
#
# Usage :
#   chmod +x scripts/setup_geographiclib.sh
#   sudo bash scripts/setup_geographiclib.sh

set -e

GEOID_PATH="/usr/share/GeographicLib/geoids/egm96-5.pgm"
GEOID_ALT_PATH="/usr/local/share/GeographicLib/geoids/egm96-5.pgm"

echo "=== GeographicLib — vérification du dataset EGM96 ==="

if [ -f "$GEOID_PATH" ] || [ -f "$GEOID_ALT_PATH" ]; then
    echo "[OK] egm96-5.pgm déjà présent — rien à faire."
    exit 0
fi

echo "[INFO] Dataset absent. Installation en cours..."

# Méthode 1 — paquet apt (recommandé)
if ! dpkg -l geographiclib-tools &>/dev/null; then
    echo "[INFO] Installation de geographiclib-tools..."
    apt-get update -q
    apt-get install -y geographiclib-tools
fi

# Méthode 1a — commande fournie par le paquet
if command -v geographiclib-get-geoids &>/dev/null; then
    echo "[INFO] Téléchargement via geographiclib-get-geoids..."
    geographiclib-get-geoids egm96-5
elif [ -f /usr/share/GeographicLib/scripts/geographiclib-get-geoids ]; then
    echo "[INFO] Téléchargement via script installé..."
    bash /usr/share/GeographicLib/scripts/geographiclib-get-geoids egm96-5
else
    # Méthode 2 — script officiel MAVROS (fallback)
    echo "[INFO] Fallback : script officiel MAVROS..."
    TMP=$(mktemp)
    wget -q -O "$TMP" \
        https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    bash "$TMP"
    rm -f "$TMP"
fi

# Vérification finale
if [ -f "$GEOID_PATH" ] || [ -f "$GEOID_ALT_PATH" ]; then
    echo "[OK] egm96-5.pgm installé avec succès."
    echo "     Chemin : $(find /usr /usr/local -name egm96-5.pgm 2>/dev/null | head -1)"
else
    echo "[ERREUR] Installation échouée. Vérifier la connexion réseau."
    exit 1
fi
