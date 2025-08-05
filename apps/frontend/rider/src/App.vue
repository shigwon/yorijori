<script setup>
import { useAppState } from './composables/useAppState'
import HowToUseScreen from './components/01_HowToUseScreen.vue'
import ScanOptionScreen from './components/02_ScanOptionScreen.vue'
import ReceiptScanScreen from './components/03_ReceiptScanScreen.vue'
import ManualInputScreen from './components/04_ManualInputScreen.vue'
import ManualConfirmScreen from './components/05_ManualConfirmScreen.vue'

const { currentScreen, progressPercent } = useAppState()
</script>

<template>
  <!-- 진행률 바 -->
  <div class="progress-bar-wrapper">
    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
    </div>
  </div>
  
  <div class="app-modal">
    <HowToUseScreen v-if="currentScreen === 'how-to-use'" />
    <ScanOptionScreen v-else-if="currentScreen === 'scan-option'" />
    <ReceiptScanScreen v-else-if="currentScreen === 'receipt-scan'" />
    <ManualInputScreen v-else-if="currentScreen === 'manual-input'" />
    <ManualConfirmScreen v-else-if="currentScreen === 'manual-confirm'" />
  </div>
</template>

<style>
/* 기존 스타일 유지 */
.progress-bar-wrapper {
  width: 90%;
  padding: 0;
  margin: 10px auto 0 auto;
  background: transparent;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  z-index: 1000;
}

.progress-bar {
  width: 100%;
  height: 2px;
  background: #E5E7EB;
  border-radius: 1px;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  background: #7C3AED;
  border-radius: 2px;
  transition: width 0.3s ease;
}

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

html, body {
  margin: 0 !important;
  padding: 0 !important;
  height: 100vh !important;
  width: 100vw !important;
  overflow: hidden !important;
  background: white !important;
}

.app-modal {
  width: 100vw;
  height: 100vh;
  background: white;
  display: flex;
  flex-direction: column;
  align-items: stretch;
  justify-content: flex-start;
  margin: 0;
  overflow: hidden;
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  padding: 0;
}

@media (max-width: 480px) {
  .app-modal {
    width: 100vw;
    height: 100vh;
    margin: 0;
    padding: 0;
  }
}
</style>