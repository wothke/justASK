# justASK

Poor man's amplitude-shift keying (ASK) driver.

The implementation is based on https://wireless.murata.com/media/products/apnotes/tr_swg05.pdf?ref=rfm.com
(the PDF can be found in the "docs" folder). With specific add-ons for limited compatibility with RadioHead's RH_ASK.

The lib has been tested with the cheap modules that can be found on AliExpress for about 1$ the pair:
![alt text](https://github.com/wothke/justASK/raw/master/docs/ASK.jpg "ASK transmitters/receivers")


If you are looking for a more portable/comprehensive implementation you might want to use RH_ASK
from the "RadioHead" library instead - which wasn't an option for me due to licensing concerns.

Personally I prefer to NOT use anything that comes with a shitty GPL license as a base
for my work. (I don't like GPL for the same reason that I don't hand out blanko checks.)
Since I didn't find anything acceptable, I decided to write my own MickyMouse ASK library 
instead. (I feel that it would be stupid to let some trivial functionality library - 
like this one - dictate the licenses that I can or cannot use for my own work.. in particular
regarding potential commercial exploitation of my work by 3rd parties.)

Copyright (C) 2018 Juergen Wothke


## Known limitations

* This implementation is targeted at a very limited scope "use case" of mine where senders
(sensors) broadcast their data and messages are received by "relay stations" that then forward the
data using other transmission technologies. Functionality may be reduced as compared to RadioHead's RH_ASK
* Has only been tested with ATmega128, ATmega328P and "Wemos D1 mini pro" (ESP8266 based).
* The Sloeber 3.0 IDE has been used. The code might not work with old Arduino IDEs and
no effort whatsoever has been made in that regard.

## License
Terms of Use: This software is licensed under a CC BY-NC-SA (http://creativecommons.org/licenses/by-nc-sa/4.0/). Commercial
licenses available on request. 
	