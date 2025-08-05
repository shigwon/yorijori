<template>
  <div id="app">
    <WelcomeScreen v-if="currentScreen === 'welcome'" @start="handleStart" />
    <HowToUseScreen v-if="currentScreen === 'how-to-use'" @next="handleNext" />
    <TermsAgreementScreen v-if="currentScreen === 'terms'" @next="handleTermsNext" />
  </div>
</template>

<script setup>
import { ref } from 'vue'
import WelcomeScreen from './components/01_WelcomeScreen.vue'
import HowToUseScreen from './components/03_HowToUseScreen.vue'
import TermsAgreementScreen from './components/04_TermsAgreementScreen.vue'

const currentScreen = ref('welcome')

const handleStart = () => {
  currentScreen.value = 'how-to-use'
}

const handleNext = () => {
  currentScreen.value = 'terms'
}

const handleTermsNext = () => {
  console.log('약관 동의 완료')
}
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin: 0;
  padding: 0;
}

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}
</style>