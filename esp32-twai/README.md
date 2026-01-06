# RTC Example

## Esporre la porta USB via rete (sul Mac):

```
esp_rfc2217_server -v -p 4000 /dev/cu.usbserial-0001
```

## Lanciare i comandi dal Docker:

Build: usare il bottone di VSCode


Monitor
```
idf.py --port 'rfc2217://host.docker.internal:4000?ign_set_control' monitor
```

Flash
```
idf.py --port 'rfc2217://host.docker.internal:4000?ign_set_control' flash
```

Salvare l'output in un file log
```
idf.py --port 'rfc2217://host.docker.internal:4000?ign_set_control' monitor > log_can_message.txt
```