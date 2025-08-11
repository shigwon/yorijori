<template>
  <div class="how-to-use-container">
    <!-- 헤더 섹션 -->
    <div class="header-section">
      <h1 class="brand-title">
        <span class="brand-name">LiNKY</span>
        <span class="title-text">이용방법!</span>
      </h1>
      <p class="subtitle">배송진행을 위해 아래 단계를 따라주세요.</p>
    </div>

    <!-- 단계 섹션 -->
    <div class="steps-section">
      <!-- 1단계: 영수증 스캔 -->
      <div class="step-item">
        <div class="step-icon">
          <div class="icon-circle">
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <rect x="6" y="4" width="12" height="16" rx="2" stroke="white" stroke-width="2" fill="none"/>
              <line x1="8" y1="8" x2="16" y2="8" stroke="white" stroke-width="2"/>
              <line x1="8" y1="11" x2="16" y2="11" stroke="white" stroke-width="2"/>
              <line x1="8" y1="14" x2="14" y2="14" stroke="white" stroke-width="2"/>
            </svg>
          </div>
        </div>
        <div class="step-content">
          <h3 class="step-title">1. 영수증 스캔</h3>
          <p class="step-description">자동스캔시 5여초 정도가 소요될 수 있습니다</p>
        </div>
      </div>

      <!-- 2단계: 위치 정보 -->
      <div class="step-item">
        <div class="step-icon">
          <div class="icon-circle">
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7z" stroke="white" stroke-width="2" fill="none"/>
              <circle cx="12" cy="9" r="2.5" stroke="white" stroke-width="2" fill="none"/>
            </svg>
          </div>
        </div>
        <div class="step-content">
          <h3 class="step-title">2. 위치정보</h3>
          <p class="step-description">고객에게 위치정보를 보내주세요</p>
        </div>
      </div>

      <!-- 3단계: 사진 촬영 -->
      <div class="step-item">
        <div class="step-icon">
          <div class="icon-circle">
            <img src="../assets/camera.png" alt="카메라" class="camera-icon" />
          </div>
        </div>
        <div class="step-content">
          <h3 class="step-title">3. 사진</h3>
          <p class="step-description">음식을 놓은 로봇 사진을 찍어주세요</p>
        </div>
      </div>
    </div>

    <!-- 하단 버튼 -->
    <div class="footer-section">
      <button class="next-button" @click="handleNext">다 음</button>
    </div>
  </div>
</template>

<script setup>
import { useAppState } from '../composables/useAppState'
import { onMounted } from 'vue'

const { goToScanOption, setProgressPercent } = useAppState()

onMounted(() => {
  setProgressPercent(5)
})

const handleNext = () => {
  goToScanOption()
}

const handleScanClick = () => {
  goToScanOption()
}
</script>

<style scoped>



.progress-bar-wrapper {
  width: 85%;
  max-width: 430px;
  height: 4px;
  background: #f3f3f3;
  margin: 24px auto 16px auto;
  border-radius: 2px;
  box-sizing: border-box;
  display: flex;
  align-items: center;
  overflow: hidden;
}
.progress-bar {
  width: 5%;
  height: 100%;
  background: #a78bfa;
  border-radius: 2px 0 0 2px;
  margin: 0;
  transition: width 0.3s;
}
.how-to-use-container {
  min-height: 100vh;
  width: 100%;
  background: white;
  display: flex;
  flex-direction: column;
  padding: 0px 24px 24px 24px;
  box-sizing: border-box;
  margin-top: -20px;
}

/* 헤더 섹션 */
.header-section {
  text-align: center;
  margin-bottom: 15px;
  margin-top: 10px;
  width: 100%;
}

.brand-title {
  margin-bottom: 15px;
  margin-top: 0px;
  display: flex;
  flex-direction: row;
  align-items: center;
  justify-content: center;
  gap: 8px;
}

.brand-name {
  font-size: 26px;
  font-weight: 700;
  color: #7C3AED;
  margin-right: 6px;
}

.title-text {
  font-size: 26px;
  font-weight: 600;
  color: #1F2937;
}

.subtitle {
  font-size: 16px;
  color: #6B7280;
  line-height: 1.5;
  margin-bottom: 0;
  margin-top: 10px;
}

/* 단계 섹션 */
.steps-section {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 60px;
  width: 100%;
  margin-top: 60px;
  position: relative;
}

/* 세로선 */
.steps-section::before {
  content: '';
  position: absolute;
  left: 30px;
  top: 60px;
  bottom: 60px;
  width: 2px;
  background: #E5E7EB;
  z-index: 1;
}

.step-item {
  display: flex;
  align-items: flex-start;
  gap: 20px;
  position: relative;
  z-index: 2;
}

.step-icon {
  flex-shrink: 0;
  position: relative;
}

.icon-circle {
  width: 60px;
  height: 60px;
  background: #A78BFA;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(167, 139, 250, 0.3);
}

.camera-icon {
  width: 24px;
  height: 24px;
  object-fit: contain;
  filter: brightness(0) invert(1); /* 이미지를 흰색으로 변환 */
}

.step-content {
  flex: 1;
  padding-top: 8px;
}

.step-title {
  font-size: 18px;
  font-weight: 700;
  color: #1F2937;
  margin: 0 0 8px 0;
  line-height: 1.3;
  text-align: left;
}

.step-description {
  font-size: 14px;
  color: #6B7280;
  line-height: 1.5;
  margin: 0;
  text-align: left;
}

/* 하단 섹션 */
.footer-section {
  width: 100%;
  padding: 20px 0;
  margin-top: 100px;
}

.next-button {
  width: 100%;
  height: 56px;
  background: linear-gradient(135deg, #7C3AED 0%, #6D28D9 100%);
  border: none;
  border-radius: 16px;
  color: white;
  font-size: 18px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  box-shadow: 0 4px 20px rgba(124, 60, 237, 0.2);
}

.next-button:hover {
  background: #6D28D9;
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(124, 60, 237, 0.3);
}

.next-button:active {
  transform: translateY(0);
}

/* 반응형 디자인 */
@media (max-width: 480px) {
  .how-to-use-container {
    padding: 80px 16px 100px 16px;
    width: 100vw;
    height: 100vh;
    box-sizing: border-box;
  }
  
  .header-section {
    margin-bottom: 35px;
    margin-top: 25px;
  }
  
  .brand-title {
    gap: 4px;
    margin-bottom: 12px;
    margin-top: 0px;
  }
  
  .brand-name {
    font-size: 24px;
    margin-right: 4px;
    margin-top: -50px;
  }
  
  .title-text {
    font-size: 24px;
    margin-top: -50px;
  }
  
  .subtitle {
    font-size: 14px;
    margin-top: -10px;
    
  }
  
  .steps-section {
    gap: 48px;
    margin-top: 35px;
  }
  
  .steps-section::before {
    left: 25px;
    top: 50px;
    bottom: 50px;
  }
  
  .step-item {
    gap: 16px;
  }
  
  .icon-circle {
    width: 50px;
    height: 50px;
  }
  
  .step-title {
    font-size: 16px;
    margin-bottom: 6px;
  }
  
  .step-description {
    font-size: 13px;
  }
  
  .footer-section {
    padding: 20px 0;
    margin-top: 80px;
  }
}
</style> 