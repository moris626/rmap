#ifndef _GSM_CONFIG_H
#define _GSM_CONFIG_H

#define GSM_APN_TIM                       ("ibox.tim.it")
#define GSM_APN_WIND                      ("internet.wind")
#define GSM_APN_VODAFONE                  ("m2mbis.vodafone.it")

#define GSM_DEFAULT_APN                   (GSM_APN_TIM)
#define GSM_DEFAULT_USERNAME              ("")
#define GSM_DEFAULT_PASSWORD              ("")

#define GSM_APN_LENGTH                    (20)
#define GSM_USERNAME_LENGTH               (20)
#define GSM_PASSWORD_LENGTH               (20)

#define GSM_TIMEOUT_MS                    (30000)
#define GSM_ATTEMPT_MS                    (2000)

#define USE_SIM_800C                      (true)
#define USE_SIM_800L                      (false)

#if (USE_SIM_800C == true && USE_SIM_800L == true)
#error What SIM800 do you want to use? specific it in gsm_config.h

#elif (USE_SIM_800C == true)
#define SIM800_ON_OFF_PIN                 (5)

#elif (USE_SIM_800L == true)
#define SIM800_ON_OFF_PIN                 (7)
#define SIM800_RESET_PIN                  (6)

#endif

#endif
