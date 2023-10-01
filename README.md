# sensor_synchronization

sensor_synchronization

## Last notes - 1.10.2023

- main working version: copy_realtime.py
- ob objavi malo urediti repository :)
- main data analysis: `data_analysis/error_analysis.py`
- podatki shranjeni v `alldata`

## Latest version - nujno ohraniti (komentarji noter):

sync test/realData/take3  
rtde-2.6.0/mycode/Datareader2

## To Do:

### Further work

- [x] real-time synchronization_test.py
- [x] two-way rtde exchange (real-time robot controls)
- [x] sensor setup
- [x] multithreading
- [x] parallel sensor reference read
- [x] live error readout
- [ ] live graphs
- [x] several cycles for reference - sort array by time - just using lower speed
- [ ] collision detection
- [x] adaptive reference if there is no collision
- [x] saving measurements
- [x] all sensor read reference motion from same feed
- [ ] if not reduced speed, morda poslji command samo ce ni overall
- [x] napake (poz/neg)
- [ ] napake (abs/rel)
- [x] safe-distance variable (1m)
- [ ] ignoriranje varnosti v delu cikla
- [ ] obdelovanje status code-ov - manjka status1 in 2
- [ ] izmerit splosen noise v meritvah senzorjev
- [ ] prilagajanje hitrosti sorazmerno odstopanju
- [x] obdelava podatkov za board 0 in 1 (zacetek, skatla not, skatla vn)
- [x] fix time measurements - niso samo narascajoca - min distance iscemo po celem ref arrayu.. morda samo za max 0.1s referencnega casa
      naprej? tudi zdaj sprejemamo DO bite in takoj ko je new cycle nastavimo prev_time na 0? morda pa v manhattan distance vkljucimo tudi cas?

### Issues

- [ ] sampling time, refresh rate? (not precise?)
- [x] speed 100 na zacetku ne dela?
- [ ] array of sample points empty in adaptacija čisto na začetku/koncu?
