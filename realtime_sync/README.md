# Seznam

## Stvari, ki jih program upošteva

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
- Thread za izračun pozicije v ciklu se izvaja v ozadju in sporoča ostalim senzorskim threadom.
