# sensor_synchronization
sensor_synchronization
## Latest version - nujno ohraniti (komentarji noter):
sync test/realData/take3  
rtde-2.6.0/mycode/Datareader2
## To Do: 
### Further work
- [X] real-time synchronization_test.py
- [X] two-way rtde exchange (real-time robot controls)  
- [X] sensor setup  
- [X] multithreading 
- [X] parallel sensor reference read 
- [X] live error readout  
- [ ] live graphs  
- [X] several cycles for reference - sort array by time - just using lower speed
- [ ] collision detection
- [ ] adaptive reference if there is no collision   
- [ ] saving measurements 
- [X] all sensor read reference motion from same feed
- [ ] if not reduced speed, morda poslji command samo ce ni overall
- [ ] napake (poz/neg) (abs/rel)
- [X] safe-distance variable (1m)
- [ ] ignoriranje varnosti v delu cikla
- [ ] obdelovanje status code-ov - manjka status1 in 2
- [ ] izmerit splosen noise v meritvah senzorjev 
- [ ] prilagajanje hitrosti sorazmerno odstopanju
- [ ] obdelat podatke za board 0 in 1 (zacetek, skatla not, skatla vn)
### Issues
- [ ] sampling time, refresh rate? (not precise?) 
- [X] speed 100 na zacetku ne dela? 
- [ ] array of sample points empty in adaptacija čisto na začetku/koncu?
