<template>
    <div class="robot-status-page">
      <!-- 헤더 -->
      <div class="page-header">
        
      </div>
  
      <!-- 메인 컨텐츠 -->
      <div class="main-content">
        <!-- 3D 뷰포트 -->
        <div class="viewport-container">
          <div class="viewport">
            <!-- 3D 자동차 모델 -->
            <div class="car-model">
              <div class="car-body">
                <div class="car-spoiler"></div>
                <div class="car-wheels">
                  <div class="wheel front-left"></div>
                  <div class="wheel front-right"></div>
                  <div class="wheel back-left"></div>
                  <div class="wheel back-right"></div>
                </div>
              </div>
            </div>
            
            <!-- 조명 효과 -->
            <div class="spotlight top-left"></div>
            <div class="spotlight bottom-right"></div>
            
            <!-- 빨간 점들 -->
            <div class="red-dot dot-1"></div>
            <div class="red-dot dot-2"></div>
            <div class="red-dot dot-3"></div>
          </div>
        </div>
  
        <!-- 컨트롤 패널 -->
        <div class="control-panel">
          <!-- Scale 섹션 -->
          <div class="control-section">
            <div class="section-header" @click="toggleSection('scale')">
              <span class="section-title">Scale</span>
              <i class="toggle-icon" :class="{ 'expanded': expandedSections.scale }">▶</i>
            </div>
            <div class="section-content" v-show="expandedSections.scale">
              <!-- Scale 컨트롤들 -->
            </div>
          </div>
  
          <!-- Rotation 섹션 -->
          <div class="control-section">
            <div class="section-header" @click="toggleSection('rotation')">
              <span class="section-title">Rotation</span>
              <i class="toggle-icon" :class="{ 'expanded': expandedSections.rotation }">▶</i>
            </div>
            <div class="section-content" v-show="expandedSections.rotation">
              <!-- Rotation 컨트롤들 -->
            </div>
          </div>
  
          <!-- Model 섹션 -->
          <div class="control-section">
            <div class="section-header" @click="toggleSection('model')">
              <span class="section-title">Model</span>
              <i class="toggle-icon" :class="{ 'expanded': expandedSections.model }">▶</i>
            </div>
            <div class="section-content" v-show="expandedSections.model">
              <!-- Model 컨트롤들 -->
            </div>
          </div>
  
          <!-- Mode 섹션 -->
          <div class="control-section">
            <div class="section-header" @click="toggleSection('mode')">
              <span class="section-title">Mode</span>
              <i class="toggle-icon" :class="{ 'expanded': expandedSections.mode }">▼</i>
            </div>
            <div class="section-content" v-show="expandedSections.mode">
              <!-- Showroom 체크박스 -->
              <div class="control-item">
                <label class="checkbox-label">
                  <input type="checkbox" v-model="controls.showroom" class="control-checkbox">
                  <span class="control-text">Showroom</span>
                </label>
              </div>
  
              <!-- Rotate 토글 -->
              <div class="control-item">
                <span class="control-text">Rotate</span>
                <div class="toggle-switch" @click="toggleRotate">
                  <div class="toggle-slider" :class="{ 'active': controls.rotate }"></div>
                </div>
              </div>
  
              <!-- Color 피커 -->
              <div class="control-item">
                <span class="control-text">Color</span>
                <input type="color" v-model="controls.color" class="color-picker">
              </div>
  
              <!-- Intensity 슬라이더 -->
              <div class="control-item">
                <span class="control-text">Intensity</span>
                <input 
                  type="range" 
                  v-model="controls.intensity" 
                  min="0" 
                  max="100" 
                  class="intensity-slider"
                >
              </div>
  
              <!-- 버튼들 -->
              <div class="control-buttons">
                <button class="control-btn" @click="randomPosition">Random Position</button>
                <button class="control-btn" @click="randomColor">Random Color</button>
                <button class="control-btn" @click="reset">Reset</button>
              </div>
            </div>
          </div>
        </div>
      </div>
  
      <!-- 하단 버튼 -->
      <div class="bottom-controls">
        <button class="close-btn">Close Co</button>
      </div>
    </div>
  </template>
  
  <script setup>
  import { ref } from 'vue'
  
  // Reactive data
  const expandedSections = ref({
    scale: false,
    rotation: false,
    model: false,
    mode: true
  })
  
  const controls = ref({
    showroom: true,
    rotate: true,
    color: '#3a57e8',
    intensity: 75
  })
  
  // Methods
  const toggleSection = (section) => {
    expandedSections.value[section] = !expandedSections.value[section]
  }
  
  const toggleRotate = () => {
    controls.value.rotate = !controls.value.rotate
  }
  
  const randomPosition = () => {
    console.log('Random Position clicked')
    // 랜덤 위치 로직
  }
  
  const randomColor = () => {
    const colors = ['#ff4757', '#3a57e8', '#13c572', '#f59e0b', '#ec4899']
    controls.value.color = colors[Math.floor(Math.random() * colors.length)]
  }
  
  const reset = () => {
    controls.value = {
      showroom: true,
      rotate: true,
      color: '#3a57e8',
      intensity: 75
    }
  }
  </script>
  
  <style scoped>
  .robot-status-page {
    
    height: 100%;
    background-color: #081028;
    color: #ffffff;
    display: flex;
    flex-direction: column;
  }
  
  /* 헤더 */
  .page-header {
    margin-bottom: 20px;
  }
  
  .page-title {
    font-size: 24px;
    font-weight: bold;
    margin: 0;
    color: #ffffff;
  }
  
  /* 메인 컨텐츠 */
  .main-content {
    display: flex;
    flex: 1;
    gap: 20px;
    margin-bottom: 20px;
  }
  
  /* 3D 뷰포트 */
  .viewport-container {
    flex: 1;
    background-color: #1a1f2e;
    border-radius: 12px;
    overflow: hidden;
  }
  
  .viewport {
    width: 100%;
    height: 100%;
    min-height: 500px;
    background-color: #0f141f;
    position: relative;
    display: flex;
    align-items: center;
    justify-content: center;
  }
  
  /* 자동차 모델 */
  .car-model {
    position: relative;
    width: 200px;
    height: 100px;
  }
  
  .car-body {
    width: 100%;
    height: 60px;
    background: linear-gradient(135deg, #8b5cf6, #a78bfa);
    border-radius: 20px 20px 8px 8px;
    position: relative;
    box-shadow: 0 8px 16px rgba(0, 0, 0, 0.3);
  }
  
  .car-spoiler {
    position: absolute;
    top: -8px;
    right: 20px;
    width: 40px;
    height: 8px;
    background-color: #6366f1;
    border-radius: 4px;
  }
  
  .car-wheels {
    position: absolute;
    bottom: -15px;
    width: 100%;
    display: flex;
    justify-content: space-between;
  }
  
  .wheel {
    width: 20px;
    height: 20px;
    background-color: #374151;
    border-radius: 50%;
    border: 2px solid #1f2937;
  }
  
  /* 조명 효과 */
  .spotlight {
    position: absolute;
    width: 100px;
    height: 100px;
    background: radial-gradient(circle, rgba(255, 255, 0, 0.3) 0%, transparent 70%);
    border-radius: 50%;
  }
  
  .spotlight.top-left {
    top: 20px;
    left: 20px;
  }
  
  .spotlight.bottom-right {
    bottom: 20px;
    right: 20px;
  }
  
  /* 빨간 점들 */
  .red-dot {
    position: absolute;
    width: 8px;
    height: 8px;
    background-color: #ff4757;
    border-radius: 50%;
  }
  
  .dot-1 {
    top: 30px;
    left: 50%;
    transform: translateX(-50%);
  }
  
  .dot-2 {
    top: 50%;
    right: 40px;
    transform: translateY(-50%);
  }
  
  .dot-3 {
    bottom: 60px;
    right: 60px;
  }
  
  /* 컨트롤 패널 */
  .control-panel {
    width: 300px;
    background-color: #222738;
    border-radius: 12px;
    padding: 20px;
    display: flex;
    flex-direction: column;
    gap: 16px;
  }
  
  .control-section {
    border-bottom: 1px solid #2a2f3e;
    padding-bottom: 12px;
  }
  
  .section-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    cursor: pointer;
    padding: 8px 0;
    transition: color 0.3s ease;
  }
  
  .section-header:hover {
    color: #3a57e8;
  }
  
  .section-title {
    font-weight: 600;
    font-size: 14px;
  }
  
  .toggle-icon {
    font-size: 12px;
    transition: transform 0.3s ease;
  }
  
  .toggle-icon.expanded {
    transform: rotate(90deg);
  }
  
  .section-content {
    padding-top: 12px;
    display: flex;
    flex-direction: column;
    gap: 12px;
  }
  
  /* 컨트롤 아이템들 */
  .control-item {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 4px 0;
  }
  
  .control-text {
    font-size: 12px;
    color: #b6bace;
  }
  
  .control-checkbox {
    width: 16px;
    height: 16px;
    accent-color: #3a57e8;
  }
  
  /* 토글 스위치 */
  .toggle-switch {
    width: 40px;
    height: 20px;
    background-color: #374151;
    border-radius: 10px;
    position: relative;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .toggle-switch:hover {
    background-color: #4b5563;
  }
  
  .toggle-slider {
    width: 16px;
    height: 16px;
    background-color: #ffffff;
    border-radius: 50%;
    position: absolute;
    top: 2px;
    left: 2px;
    transition: transform 0.3s ease;
  }
  
  .toggle-slider.active {
    transform: translateX(20px);
  }
  
  /* 컬러 피커 */
  .color-picker {
    width: 30px;
    height: 20px;
    border: none;
    border-radius: 4px;
    cursor: pointer;
  }
  
  /* 강도 슬라이더 */
  .intensity-slider {
    width: 100px;
    height: 4px;
    background-color: #374151;
    border-radius: 2px;
    outline: none;
    cursor: pointer;
  }
  
  .intensity-slider::-webkit-slider-thumb {
    appearance: none;
    width: 16px;
    height: 16px;
    background-color: #3a57e8;
    border-radius: 50%;
    cursor: pointer;
  }
  
  /* 컨트롤 버튼들 */
  .control-buttons {
    display: flex;
    flex-direction: column;
    gap: 8px;
    margin-top: 8px;
  }
  
  .control-btn {
    padding: 6px 12px;
    background-color: #374151;
    border: none;
    border-radius: 4px;
    color: #ffffff;
    font-size: 11px;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .control-btn:hover {
    background-color: #3a57e8;
  }
  
  /* 하단 컨트롤 */
  .bottom-controls {
    display: flex;
    justify-content: center;
  }
  
  .close-btn {
    padding: 8px 16px;
    background-color: #374151;
    border: none;
    border-radius: 6px;
    color: #ffffff;
    font-size: 12px;
    cursor: pointer;
    transition: background-color 0.3s ease;
  }
  
  .close-btn:hover {
    background-color: #4b5563;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .main-content {
      flex-direction: column;
    }
    
    .control-panel {
      width: 100%;
    }
    
    .viewport {
      min-height: 300px;
    }
    
    .car-model {
      width: 150px;
      height: 75px;
    }
  }
  
  @media (max-width: 480px) {
    .robot-status-page {
      padding: 10px;
    }
    
    .page-title {
      font-size: 20px;
    }
    
    .control-panel {
      padding: 15px;
    }
    
    .car-model {
      width: 120px;
      height: 60px;
    }
  }
  </style>