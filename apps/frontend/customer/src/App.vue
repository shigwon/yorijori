<template>
  <div id="app">
    <WelcomeScreen @start="handleStart" />
  </div>
</template>

<script setup>
import WelcomeScreen from './components/01_WelcomeScreen.vue'

const handleStart = () => {
  console.log('시작하기 버튼 클릭됨')
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