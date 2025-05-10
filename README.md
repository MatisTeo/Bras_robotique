# 🤖 Projet Bras Robotique Contrôlé par Gant Connecté

## 🎥 Démonstration vidéo
[![Regarder la vidéo sur YouTube](https://img.youtube.com/vi/8cOm21cPcBU/0.jpg)](https://www.youtube.com/watch?v=8cOm21cPcBU)

## 📘 Présentation

Ce projet consiste à développer un **bras robotique à faible coût**, capable d'exécuter :
- des **mouvements automatiques prédéfinis**
- un **contrôle manuel à distance** à l'aide d’un **gant connecté**

L'objectif est de démontrer une alternative simple et économique pour la télémanipulation dans des environnements industriels.

## 🧠 Membres du projet
- **Téo Baillot d’Estivaux** — responsable software, modélisation 3D, banc de test
- **Matis Jaquet** — responsable hardware, PCB, assemblage électronique

## 🔧 Fonctionnalités
- Contrôle **automatique** via séquences programmées
- Contrôle **manuel en temps réel** via un gant avec capteur flex + MPU-6050
- Communication **sans fil via ESP-NOW (Wi-Fi)** entre gant et bras
- Structure imprimée en 3D, légère et modulaire
- Retour d’effort par détection de courant via INA219

## 📂 Contenu du dépôt
- `Hardware/` : Schémas et PCB sous KiCad
- `Software/` : Codes Arduino pour l’émetteur (gant) et le récepteur (bras)
- `Photos/Spécification/` : Images du montage
- `Memoire_Baillot_Jaquet.pdf` : Mémoire complet du projet (ELEC4 FISA 2024/2025)

## 🛠️ Composants utilisés
- 2x ESP32
- 1x MPU-6050 (accéléromètre)
- 1x capteur flex
- 5x servomoteurs SG90
- 1x INA219
- 1x bouton poussoir
- Câblage, breadboards, batteries

## 🚀 Instructions de montage
1. Imprimer les pièces 3D du bras
2. Souder les composants sur les PCB fournis
3. Assembler le gant en cousant les composants selon le schéma
4. Flasher les codes sur les ESP32 avec Arduino IDE
5. Alimenter les deux systèmes et tester les modes automatique et manuel

## 📎 Liens utiles
- [🔗 Dépôt GitHub](https://github.com/MatisTeo/Bras_robotique)
- [📄 Mémoire complet (PDF)](./Memoire_Baillot_Jaquet.pdf)
