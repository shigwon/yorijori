<template>
  <div class="scan-option-container">
    <!-- 헤더 섹션 -->
    <div class="header-section">
      <h1 class="title">스캔 방식을 선택해주세요</h1>
      <p class="subtitle">원하시는 방식을 선택해주세요!</p>
    </div>

    <!-- 옵션 섹션 -->
    <div class="options-section">
      <!-- 자동 스캔 옵션 -->
      <div class="option-card auto-card" @click="handleAutoScan">
        <div class="card-header">
          <span class="card-label">자동</span>
        </div>
        <div class="card-content">
          <div class="option-icon">
            <img src="../assets/automatic scan.png" alt="Automatic Scan" class="automatic-scan-image" />
          </div>
          <div class="card-text-content">
            <h3 class="card-title">영수증 촬영으로 자동 스캔</h3>
            <p class="card-benefit">카메라로 영수증을 촬영하면 자동으로 주문정보를 인식해요</p>
          </div>
        </div>
        <button class="card-button">스캔하기</button>
      </div>

      <!-- 수동 스캔 옵션 -->
      <div class="option-card manual-card" @click="handleManualScan">
        <div class="card-header">
          <span class="card-label">수동</span>
        </div>
        <div class="card-content">
          <div class="option-icon">
            <img src="../assets/manual input.png" alt="Manual Input" class="manual-input-image" />
          </div>
          <div class="card-text-content">
            <h3 class="card-title">직접 입력으로 수동 등록</h3>
            <p class="card-benefit">주문번호와 안심번호를 직접 입력해서 등록할 수 있어요</p>
          </div>
        </div>
        <button class="card-button">입력하기</button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { onMounted } from 'vue'

const { goToReceiptScan, goToManualInput, goToHowToUse, setProgressPercent } = useAppState()

onMounted(() => {
  setProgressPercent(20)
})

const handleAutoScan = () => {
  goToReceiptScan()
}

const handleManualScan = () => {
  goToManualInput()
}

const handleBack = () => {
  goToHowToUse()
}
</script>

<style scoped>
.scan-option-container {
  min-height: 100vh;
  width: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  padding: 20px 24px 24px 24px;
  box-sizing: border-box;
  position: relative;
}

/* 헤더 섹션 */
.header-section {
  text-align: center;
  margin-bottom: 80px;
  width: 100%;
  margin-top: 40px;
}

.title {
  font-size: 24px;
  font-weight: 700;
  color: #1F2937;
  margin: 0 0 8px 0;
  line-height: 1.3;
}

.subtitle {
  font-size: 16px;
  color: #6B7280;
  margin: 0;
  line-height: 1.5;
}

/* 옵션 섹션 */
.options-section {
  flex: 1;
  display: flex;
  gap: 16px;
  margin-bottom: 40px;
  justify-content: center;
  align-items: center;
  min-height: 300px;
}

.option-card {
  flex: 1;
  max-width: 100px;
  display: flex;
  flex-direction: column;
  border-radius: 16px;
  cursor: pointer;
  transition: all 0.3s ease;
  overflow: hidden;
  position: relative;
}

.option-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.15);
}

.auto-card {
  background: linear-gradient(135deg, #F3E8FF 0%, #E9D5FF 100%);
}

.manual-card {
  background: linear-gradient(135deg, #F3E8FF 0%, #E9D5FF 100%);
}

.card-header {
  padding: 3px 6px;
  display: flex;
  justify-content: flex-start;
}

.card-label {
  background: #1F2937;
  color: white;
  padding: 1px 6px;
  border-radius: 12px;
  font-size: 8px;
  font-weight: 600;
}

.card-content {
  flex: 1;
  padding: 12px 8px;
  display: flex;
  flex-direction: row;
  align-items: center;
  text-align: left;
  gap: 20px;
  height: 100%;
}

.option-icon {
  width: 35%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 0;
  flex-shrink: 0;
}

.automatic-scan-image,
.manual-input-image {
  width: 70%;
  height: 70%;
  object-fit: contain;
}

.card-text-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: flex-start;
  min-width: 0;
  height: 100%;
  justify-content: center;
}

.card-title {
  font-size: 15px;
  font-weight: 600;
  margin: 0 0 5px 0;
  line-height: 1.4;
  text-align: left;
}

.auto-card .card-title {
  color: #1F2937;
}

.manual-card .card-title {
  color: #1F2937;
}

.card-benefit {
  font-size: 12px;
  margin: 0;
  line-height: 1.3;
  text-align: left;
}

.auto-card .card-benefit {
  color: #6B7280;
}

.manual-card .card-benefit {
  color: #6B7280;
}

.card-button {
  width: 100%;
  padding: 6px 8px;
  border: none;
  border-radius: 8px;
  font-size: 10px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
  margin-top: 6px;
}

.auto-card .card-button {
  background: #8B5CF6;
  color: white;
}

.auto-card .card-button:hover {
  background: #7C3AED;
}

.manual-card .card-button {
  background: #8B5CF6;
  color: white;
}

.manual-card .card-button:hover {
  background: #7C3AED;
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .scan-option-container {
    padding: 50px 16px 80px 16px;
    min-height: 100vh;
  }
  
  .header-section {
    margin-bottom: 40px;
    margin-top: 0;
  }
  
  .title {
    font-size: 22px;
  }
  
  .subtitle {
    font-size: 16px;
  }
  
  .options-section {
    flex-direction: column;
    gap: 20px;
    align-items: center;
    min-height: auto;
    margin-bottom: 30px;
  }
  
  .option-card {
    max-width: 100%;
    width: 100%;
    min-height: 200px;
  }
  
  .card-content {
    padding: 12px 8px;
    flex: 1;
    display: flex;
    flex-direction: row;
    align-items: center;
    text-align: left;
    gap: 15px;
    height: 100%;
  }
  
  .option-icon {
    width: 35%;
    height: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    margin-bottom: 0;
    flex-shrink: 0;
    margin-top: -10px;
    margin-bottom: -10px;
  }
  
  .auto-card .option-icon {
    justify-content: flex-start;
    padding-left: 10px;
  }
  
  .card-text-content {
    flex: 1;
    display: flex;
    flex-direction: column;
    align-items: flex-start;
    min-width: 0;
    height: 100%;
    justify-content: center;
  }
  
  .card-title {
    font-size: 15px;
    margin: 0 0 5px 0;
  }
  
  .card-benefit {
    font-size: 12px;
    line-height: 1.3;
  }
  
  .card-button {
    padding: 8px 12px;
    font-size: 13px;
    margin-top: 10px;
  }
  
  .card-header {
    padding: 5px 8px;
  }
  
  .card-label {
    padding: 3px 8px;
    font-size: 11px;
  }
}
</style> 