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
      <!-- 단계 1: 사진 선택 -->
      <div class="step-item">
        <div class="step-icon">
          <div class="icon-circle">
            <img src="../assets/camera.png" alt="camera" class="camera-icon" />
          </div>
        </div>
        <div class="step-content">
          <h3 class="step-title">1. 사진을 선택하거나 촬영해주세요</h3>
          <p class="step-description">※로봇 도착 시 본인확인에 사용됩니다. 사진은 배달이 도착즉시 삭제됩니다.</p>
        </div>
      </div>

      <!-- 단계 2: 위치 -->
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
          <h3 class="step-title">2. 내 위치를 공유해주세요</h3>
          <p class="step-description">정확한 배달을 위해 필요합니다</p>
        </div>
      </div>

      <!-- 단계 3: 얼굴 인식 -->
      <div class="step-item">
        <div class="step-icon">
          <div class="icon-circle">
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <circle cx="12" cy="8" r="4" stroke="white" stroke-width="2" fill="none"/>
              <path d="M4 20c0-4.4 3.6-8 8-8s8 3.6 8 8" stroke="white" stroke-width="2" fill="none"/>
            </svg>
          </div>
        </div>
        <div class="step-content">
          <h3 class="step-title">3. 도착하면 로봇에게 얼굴을 보여주세요</h3>
          <p class="step-description">음신칸이 열리며 음식 수렴이 가능합니다.</p>
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
import { onMounted } from 'vue'
import { useAppState } from '../composables/useAppState'

const { goToTermsAgreement } = useAppState()

onMounted(() => {
  console.log('How to Use Screen mounted')
})

const handleNext = () => {
  console.log('다음 버튼 클릭됨')
  goToTermsAgreement()
}
</script>

<style scoped>
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


.steps-section {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 60px;
  width: 100%;
  margin-top: 60px;
  position: relative;
}

/* Vertical Line */
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
  filter: brightness(0) invert(1);
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
  position: relative;
  bottom: 40px;
}

.next-button:hover {
  background: #6D28D9;
  transform: translateY(-2px);
  box-shadow: 0 6px 25px rgba(124, 60, 237, 0.3);
}

.next-button:active {
  transform: translateY(0);
}

/* Responsive Design */
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
    margin-bottom: 10px;

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