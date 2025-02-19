from melo.api import TTS

import nltk
nltk.download('averaged_perceptron_tagger_eng')

# Speed is adjustable
speed = 0.8

# CPU is sufficient for real-time inference.
# You can set it manually to 'cpu' or 'cuda' or 'cuda:0' or 'mps'
device = 'auto' # Will automatically use GPU if available

# English 
text = f"I couldn't believe it—after years of hard work, I finally got the job! My heart raced with excitement as I called my mom, but just as she picked up, a lump formed in my throat, remembering Dad wasn’t here to see this moment. A sudden crash outside made me jump, fear tightening my chest. I rushed to the window, only to see my little brother laughing—he had knocked over a stack of boxes. I sighed in relief, shaking my head, when out of nowhere, my phone buzzed with a message: \"Pack your bags, we're going to Paris!\" My jaw dropped. Was this day even real?"
model = TTS(language='EN', device=device)
speaker_ids = model.hps.data.spk2id

# American accent
output_path = 'en-us.wav'
model.tts_to_file(text, speaker_ids['EN-US'], output_path, speed=speed)

# # British accent
# output_path = 'en-br.wav'
# model.tts_to_file(text, speaker_ids['EN-BR'], output_path, speed=speed)

# # Indian accent
# output_path = 'en-india.wav'
# model.tts_to_file(text, speaker_ids['EN_INDIA'], output_path, speed=speed)

# # Australian accent
# output_path = 'en-au.wav'
# model.tts_to_file(text, speaker_ids['EN-AU'], output_path, speed=speed)

# # Default accent
# output_path = 'en-default.wav'
# model.tts_to_file(text, speaker_ids['EN-Default'], output_path, speed=speed)


# import nltk
# import ssl

# try:
#     _create_unverified_https_context = ssl._create_unverified_context
# except AttributeError:
#     pass
# else:
#     ssl._create_default_https_context = _create_unverified_https_context

# nltk.download()

"