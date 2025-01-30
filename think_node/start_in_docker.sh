#!/bin/bash
ollama serve &
sleep 3
# ollama run tripplyons/r1-distill-qwen-7b
ollama run deepseek-r1:8b
wait
