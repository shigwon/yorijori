<script setup>
import { computed, ref } from 'vue'
import { useRoute } from 'vue-router'
import LoadingModal from './components/09_LoadingModal.vue'
import ScanConfirmModal from './components/10_ScanConfirmModal.vue'

const route = useRoute()

// 모달 상태 관리
const showLoadingModal = ref(false)
const showScanConfirmModal = ref(false)

// 현재 라우트의 progress 메타데이터를 기반으로 진행률 계산
const progressPercent = computed(() => {
  return route.meta.progress || 0
})

// 모달 표시 함수들
const openLoadingModal = () => {
  showLoadingModal.value = true
}

const closeLoadingModal = () => {
  showLoadingModal.value = false
}

const openScanConfirmModal = () => {
  showScanConfirmModal.value = true
}

const closeScanConfirmModal = () => {
  showScanConfirmModal.value = false
}

// 전역으로 모달 함수들을 제공
window.openLoadingModal = openLoadingModal
window.closeLoadingModal = closeLoadingModal
window.openScanConfirmModal = openScanConfirmModal
window.closeScanConfirmModal = closeScanConfirmModal
</script>

<template>
  <!-- 진행률 바 -->
  <div class="progress-bar-wrapper">
    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
    </div>
  </div>
  
  <div class="app-modal">
    <router-view />
  </div>

  <!-- 모달들 -->
  <LoadingModal v-if="showLoadingModal" @close="closeLoadingModal" />
  <ScanConfirmModal v-if="showScanConfirmModal" @close="closeScanConfirmModal" />
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