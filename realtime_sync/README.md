# Adaptive Lidar Envelope

Na robotski roki s preprostimi 3D nastavki ob sklepih namestimo lidar senzorje (2 na arduino), ki s svojimi FOV naredijo ovojnico merjenja razdalj do ovir. Te podatke za konstanten cikel lahko v realnem času uporabimo za spremljanje in reagiranje na okolico sodelujočega robota -
trenutno je to upočasnitev in prilagajanje referenčne slike pričakovanih razdalj v ovojnici na osnovi spremenjenega okolja.

## Stvari, ki jih program upošteva - "wishlist" summary

### Referenca

- Posname referenco giba robota, jo shrani ob dodani zastavici -r (v referenci tudi sile)
- Posname referenco slike senzorjev, jo shrani ob dodani zastavici -r
- Ponovno uporabi shranjene reference
- Referenco overfitta na 1000ms, ter jo vsake določeno število časa prilagodi
- Prilagajanje reference se izogne špicam z naključnim intervalom
- Prilagodimo s poljubno utežjo na novo sliko - postopoma dobimo vedno bolj natančno referenco
- V primeru zaznanih sil na robota reference ne prilagaja

### Izračun pozicije v ciklu

- Na podlagi pozicij in hitrosti sklepov izračuna pozicijo in interpolira trenutni delež cikla
- Če je robot pri miru, referenčni čas ne teče, preskoči ko se robot začne premikati.
- Thread za izračun pozicije v ciklu se izvaja v ozadju in sporoča ostalim senzorskim threadom skozi globalno spremenljivko.

### Spremljanje sil

- V sklepih na podlagi reference sil lahko spremlja prisotnost udarca v trenutnem intervalu prilagajanja reference (npr. 1s) - prepreči prilagajanje

### Branje s senzorjev

- Paralelno se bere podatke z vseh Arduinotov in se jih obdela:
- Error code handling.
- Izračuna se odstopanje od pričakovane razdalje, ter preveri ali se je to odstopanje že nekajkrat ponovilo.
- Upočasnitev robota / ponovna vzpostavitev 100% hitrosti skozi RTDE komunikacijo.
