reste securisation wifi
upgrade pour les différentes versions

Dossier Technique : Grand Prix ECU 50cc (Édition V18)

Le premier système d'assistance au pilotage de classe MotoGP conçu pour les moteurs 2 Temps à allumage PVL.
1. L'Essence du Projet

L'objectif est de concevoir une carte électronique de gestion moteur (ECU) sur mesure, basée sur un microcontrôleur ESP32 et couplée à une interface Web embarquée. Ce module dote une moto de course 50cc 2 temps, équipée d'un allumage autonome (type PVL), de technologies d'assistance habituellement réservées aux Superbikes 4 temps de pointe : Quickshifter, Launch Control, Predictive Slip Control (motricité), Pit Limiter et Télémétrie en temps réel.
2. La Subtilité Majeure : La Physique du 2T et du PVL

Appliquer de l'électronique moderne sur une mécanique "rustique" exige des choix d'ingénierie stricts :

    L'abandon du Traction Control en courbe : Sur un 2 temps à carburateur, on ne peut pas réduire la puissance en douceur (pas de Ride-by-Wire). Faire des micro-coupures brutales pendant que le pilote est sur l'angle provoquerait un retour d'adhérence violent (high-side) et des détonations dans l'échappement. Le système se concentre donc sur la motricité en ligne droite.

    L'Ignition Cut inversé : Un PVL crée sa propre étincelle, sans batterie. Pour couper le moteur, on n'ouvre pas le circuit : on le ferme en mettant l'allumage à la masse.

    Le bouclier électronique : Le PVL génère des pics de tension primaire destructeurs (jusqu'à 400V). La carte isole l'ESP32 (3.3V) via un Optocoupleur pilotant un Thyristor ou un Mosfet SiC haute tension.

    Le filtre EMI absolu : Le bruit électromagnétique d'un PVL est colossal. Le logiciel intègre un filtre mathématique qui rejette toute impulsion parasite physiquement impossible (ex: une fréquence équivalente à plus de 20 000 tr/min), garantissant un signal pur.

3. L'Arbitre Central : Le Cerveau de la Machine

Dans un calculateur de course, plusieurs assistances peuvent entrer en conflit (ex: passer une vitesse pendant le Launch Control). Pour garantir un comportement infaillible, le code est régi par un Arbitre Central à Priorité Stricte qui décide à chaque microseconde qui a le contrôle de l'étincelle :

    🔴 OVERREV LIMITER (Survie) : Limiteur "Hard" à 16 000 tr/min. Sauve le moteur en cas de casse mécanique ou d'emballement. Écrase toutes les autres commandes.

    🟡 PIT LIMITER (Règlement) : Bride la moto à la vitesse des stands (4000 tr/min).

    🔵 QUICKSHIFTER (Dynamique) : Priorité sur la motricité pour garantir un passage de rapport éclair.

    🟠 LAUNCH CONTROL (Départ) : Gestion du régime stationnaire sur la grille.

    🟢 SLIP CONTROL (Adhérence) : Gestion fine de la glisse en ligne droite.

4. Les Fonctionnalités Dynamiques

A. Le Launch & Predictive Slip Control (La Motricité)

    Phase 1 (2-Step Limiter) : À l'arrêt, embrayage tiré, gaz en grand. L'ECU maintient le moteur au régime de couple optimal (ex: 8000 tr/min) via des coupures millimétrées. Le pot d'échappement se charge en pression sans emballer la mécanique.

    Phase 2 (Predictive Slip) : Au lâcher d'embrayage (> 5 km/h), le système compare la roue avant (libre) et arrière (motrice). Subtilité MotoGP : L'algorithme ne se contente pas d'attendre que la roue patine ; il calcule la dérivée de glisse (l'accélération du patinage) pour prédire la perte d'adhérence 50 millisecondes dans le futur. Si la limite (ex: 15%) est franchie, des micro-coupures dynamiques, adoucies à basse vitesse, redonnent du grip sans étouffer le moteur.

B. Le Quickshifter (Passage à la volée)
Une pression sur le sélecteur déclenche une coupure nette (ex: 50 ms). Cette coupure annule la force de traction, libère les crabots de la boîte, et permet d'enclencher le rapport supérieur sans débrayer ni couper les gaz. Le système est protégé par un anti-rebond matériel (ISR) garantissant qu'un appui imprécis ne générera jamais de double-coupure.
5. L'Architecture Informatique : Fiabilité Aérospatiale

Un plantage logiciel à 130 km/h n'est pas permis. L'ECU repose sur un système d'exploitation Temps Réel (FreeRTOS) :

    Dual-Core Isolé : Le Core 0 gère exclusivement le réseau WiFi et la télémétrie. Le Core 1 est un sanctuaire dédié à 100% au calcul moteur. Si le WiFi sature, le moteur ne ressent aucun décalage (Zéro Jitter).

    Atomicité : Toutes les données partagées entre les cœurs sont protégées par des verrous de mémoire (std::atomic et Spinlocks).

    Double Watchdog : Des "chiens de garde" matériels surveillent la puce. Au moindre gel applicatif, le système redémarre instantanément en rendant l'allumage au pilote.

6. L'Interface Utilisateur : Application Web & Télémétrie

Pas de câbles, pas de PC encombrants. L'ESP32 génère son propre réseau WiFi sécurisé sur la grille de départ. Le pilote ou le mécanicien se connecte avec son smartphone et accède à une interface permettant :

    Le Live Data (Télémétrie) : Affichage en temps réel des RPM, des vitesses de roues, du pourcentage de glisse, et du statut de l'Arbitre Central (Shifting, Slip, Pit, etc.).

    Le Flashage NVRAM : Ajustement des paramètres (temps de cut shifter, régime launch, tolérance de glisse) appliqués instantanément et sauvegardés à vie dans la mémoire non-volatile de la puce.


Manuel Technique : Grand Prix ECU 50cc (Édition V18)

Le premier système d'assistance au pilotage de classe MotoGP conçu pour les moteurs 2 Temps à allumage autonome (PVL).
1. La Philosophie : Pourquoi ce design ?

Appliquer l'électronique moderne d'une Superbike 1000cc sur un moteur 50cc 2 Temps à carburateur est un défi physique et électrique majeur. Ce système n'est pas un simple "gadget Arduino", c'est un calculateur temps-réel conçu pour survivre à un environnement hostile.

    Le défi du 2 Temps : Contrairement à un 4 Temps moderne (Ride-by-Wire), on ne peut pas fermer les papillons des gaz électroniquement pour réduire la puissance. La seule arme dont nous disposons est la coupure d'allumage (Ignition Cut).

    La spécificité du PVL : Un allumage PVL de course est autonome (sans batterie) et génère des tensions primaires extrêmes. Pour couper le moteur, on ne coupe pas l'alimentation : on met le système à la masse via un Switch de puissance haute tension.

    L'enfer Électromagnétique (EMI) : Un moteur de course génère des parasites colossaux. L'ECU intègre un filtre logiciel absolu : il rejette mathématiquement toute impulsion d'allumage physiquement impossible (au-delà de 20 000 tr/min), garantissant que le cerveau électronique ne paniquera jamais face aux interférences.

2. L'Arbitre Central : Le Cerveau de la Machine

Dans un calculateur de course, plusieurs assistances peuvent vouloir agir en même temps (ex: le pilote passe une vitesse alors qu'il est dans les stands). Pour éviter les conflits et les crashs logiciels, le système repose sur une Machine à États à Priorité Stricte. À chaque microseconde, l'Arbitre Central décide qui a le contrôle de l'étincelle :

    🔴 Priorité 1 - OVERREV LIMITER (Survie Moteur) : Si le régime dépasse 16 000 tr/min, l'ECU coupe tout. C'est le filet de sécurité ultime en cas de casse de la transmission ou de capteur défaillant.

    🟡 Priorité 2 - PIT LIMITER (Règlement Course) : Activé au guidon, il bride la moto à 4000 tr/min dans la voie des stands pour éviter les pénalités, écrasant toute autre fonction d'accélération.

    🔵 Priorité 3 - QUICKSHIFTER (Dynamique) : Permet de monter les rapports à pleine charge. Il prend la priorité sur la motricité pour garantir que la boîte de vitesses soit libérée en moins de 50 millisecondes.

    🟠 Priorité 4 - LAUNCH CONTROL (Départ Arrêté) : Moto à l'arrêt, embrayage tiré. L'ECU maintient le régime parfait (ex: 8000 tr/min) pour charger l'échappement en pression sans emballer le moteur.

    🟢 Priorité 5 - PREDICTIVE SLIP CONTROL (Motricité) : En piste, il gère la glisse de la roue arrière pour propulser la moto sans perte d'adhérence.

3. Les Assistances au Rayon X : Comment ça marche ?
⚡ Quickshifter (Passage à la volée)

Le capteur du sélecteur est lu par une interruption matérielle (ISR) dotée d'un "debounce" ultra-rapide pour ignorer les rebonds mécaniques de la boîte. L'ECU offre une fenêtre d'action de 25 millisecondes pour analyser l'appui du pilote. S'il est valide et que le régime est suffisant, il ordonne une coupure nette (ex: 50 ms), permettant aux crabots de la boîte de s'engager sans débrayer ni couper les gaz.
🚀 Predictive Slip Control (L'Anticipation de la glisse)

C'est la pièce maîtresse du système. Dès que la moto dépasse 5 km/h, l'ECU compare en permanence la vitesse de la roue avant (libre) et de la roue arrière (motrice) grâce à des capteurs magnétiques.

    La Prédiction : Au lieu d'attendre que la moto patine pour réagir, l'algorithme calcule la dérivée de glisse (l'accélération du patinage). Il peut ainsi prédire l'adhérence à +50 millisecondes dans le futur.

    L'Action : S'il détecte une amorce de patinage excessif (ex: > 15%), il ne coupe pas brutalement. Il applique des micro-coupures variables (de 5 à 25 ms) modulées par la vitesse de la moto. Résultat : une accélération parfaitement lissée et redoutable en sortie de courbe.

⏱️ Télémétrie et Interface Web Embarquée

Fini les câbles et les ordinateurs portables sur la grille de départ. L'ECU génère son propre réseau WiFi sécurisé. Les mécaniciens se connectent avec un smartphone et accèdent à un tableau de bord en temps réel.

    Live Data : Affichage des RPM, des vitesses de roue, du pourcentage de glisse et du "Mode" actif de l'Arbitre Central (via un badge de couleur instantané).

    Live Mapping : Modification immédiate des temps de coupure, des régimes limites, de la circonférence des pneus et de la tolérance du Slip Control. Les données sont flashées en NVRAM et mémorisées à vie.

4. L'Architecture Aérospatiale (Fiabilité Absolue)

Un plantage informatique à 14 000 tr/min à 130 km/h n'est pas une option. Le code a été forgé avec des standards de l'industrie critique :

    Dual-Core asymétrique : L'ESP32 possède deux cœurs. Le Core 0 est dédié exclusivement au WiFi et à la Télémétrie. Le Core 1 est un "sanctuaire" dédié uniquement au calcul moteur temps-réel, garantissant zéro latence (Jitter).

    Atomicité et Spinlocks : Toutes les variables partagées (vitesse, RPM) utilisent la bibliothèque std::atomic et des barrières mémoire (portENTER_CRITICAL). Il est mathématiquement impossible que le module "lise" une vitesse à moitié calculée.

    Accès Direct aux Registres : L'ordre de couper l'allumage ne passe pas par les fonctions classiques d'Arduino, mais tape directement dans les registres du silicium (GPIO.out_w1ts), déclenchant la coupure en 1 seul cycle d'horloge (12 nanosecondes).

    Double Watchdog : Des "chiens de garde" matériels surveillent les deux cœurs de la puce. Si le WiFi freeze ou si le calcul moteur prend plus de 3 secondes (impossible en temps normal), la puce redémarre instantanément pour rendre le contrôle au pilote.