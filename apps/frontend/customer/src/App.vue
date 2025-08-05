<template>
  <div id="app">
    <WelcomeScreen v-if="currentScreen === 'welcome'" @start="handleStart" />
    <HowToUseScreen v-if="currentScreen === 'how-to-use'" @next="handleNext" />
  </div>
</template>

<script setup>
import { ref } from 'vue'
import WelcomeScreen from './components/01_WelcomeScreen.vue'
import HowToUseScreen from './components/02_HowToUseScreen.vue'

const currentScreen = ref('welcome')

const handleStart = () => {
  currentScreen.value = 'how-to-use'
}

const handleNext = () => {
  console.log('다음 화면으로 이동')
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