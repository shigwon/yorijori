<template>
    <div class="login-container">
     
  
      <!-- 로그인 폼 -->
      <div class="login-form">
        <!-- 로고 영역 -->
        <div class="logo-section">
          <div class="logo-placeholder">
            <i class="logo-icon">👤</i>
          </div>
        </div>
  
        <!-- 로그인 제목 -->
        <h1 class="login-title">Log in</h1>
  
        <!-- 에러 메시지 -->
        <div v-if="errorMessage" class="error-message">
          {{ errorMessage }}
        </div>
  
        <!-- 성공 메시지 -->
        <div v-if="successMessage" class="success-message">
          {{ successMessage }}
        </div>
  
        <!-- 입력 폼 -->
        <form @submit.prevent="handleLogin" class="form">
          <!-- 이메일/사용자명 입력 -->
          <div class="input-group">
            <label for="email" class="input-label">Email address or user name</label>
            <input
              id="email"
              v-model="formData.email"
              type="text"
              class="input-field"
              placeholder="Enter your email or username"
              required
            />
          </div>
  
          <!-- 비밀번호 입력 -->
          <div class="input-group">
            <label for="password" class="input-label">Password</label>
            <div class="password-container">
              <input
                id="password"
                v-model="formData.password"
                :type="showPassword ? 'text' : 'password'"
                class="input-field password-input"
                placeholder="Enter your password"
                required
              />
              <button
                type="button"
                class="password-toggle"
                @click="togglePassword"
              >
                {{ showPassword ? 'Hide' : 'Show' }}
              </button>
            </div>
          </div>
  
          <!-- 기억하기 체크박스 -->
          <div class="checkbox-group">
            <label class="checkbox-label">
              <input
                v-model="formData.rememberMe"
                type="checkbox"
                class="checkbox"
              />
              <span class="checkbox-text">Remember me</span>
            </label>
          </div>
  
          <!-- 이용약관 동의 -->
          <div class="terms-text">
            By continuing, you agree to the 
            <a href="#" class="terms-link">Terms of use</a> 
            and 
            <a href="#" class="terms-link">Privacy Policy</a>.
          </div>
  
          <!-- 로그인 버튼 -->
          <button type="submit" class="login-button" :disabled="isLoading">
            <span v-if="isLoading" class="loading-spinner"></span>
            {{ isLoading ? 'Logging in...' : 'Log in' }}
          </button>
  
          <!-- 추가 링크들 -->
          <div class="additional-links">
            <a href="#" class="link">Forget your password</a>
            
          </div>
        </form>
      </div>
  
      <!-- 우측 검은 영역 -->
      
    </div>
  </template>
  
  <script setup>
  import { ref, onMounted } from 'vue'
  import { useRouter } from 'vue-router'
  import axios from 'axios'
  
  const router = useRouter()
  
  // Reactive data
  const formData = ref({
    email: '',
    password: '',
    rememberMe: false
  })
  const showPassword = ref(false)
  const isLoading = ref(false)
  const errorMessage = ref('')
  const successMessage = ref('')
  
  // Methods
  const togglePassword = () => {
    showPassword.value = !showPassword.value
  }
  
  // 에러 메시지 표시
  const showError = (message) => {
    errorMessage.value = message
    successMessage.value = ''
    setTimeout(() => {
      errorMessage.value = ''
    }, 2000)
  }
  
  // 성공 메시지 표시
  const showSuccess = (message) => {
    successMessage.value = message
    errorMessage.value = ''
    setTimeout(() => {
      successMessage.value = ''
    }, 300)
  }
  
  // 입력값 검증
  const validateForm = () => {
    if (!formData.value.email.trim()) {
      showError('이메일을 입력해주세요.')
      return false
    }
    
    if (!formData.value.password.trim()) {
      showError('비밀번호를 입력해주세요.')
      return false
    }
    
    return true
  }
  
  const handleLogin = async () => {
    // 입력값 검증
    if (!validateForm()) {
      return
    }
    
    isLoading.value = true
    errorMessage.value = ''
    successMessage.value = ''
    
    try {
      // API 호출
      const { data } = await axios.post('/api/admin/login', {
        email: formData.value.email,
        password: formData.value.password
      })
  
      // 로그인 성공
      showSuccess(data.message || '로그인 성공! 관리자 페이지로 이동합니다.')
      
      // 로그인 정보 저장 (Remember me 체크 시)
      if (formData.value.rememberMe) {
        localStorage.setItem('adminEmail', formData.value.email)
      }
      
      // 즉시 페이지 이동 (UX 개선)
      setTimeout(() => {
        router.push('/dashboard-detail')
      }, 500)
      
    } catch (error) {
      console.error('로그인 오류:', error)
      
      // axios 에러 처리
      if (error.response) {
        // 서버에서 에러 응답 (4xx, 5xx)
        const errorMessage = error.response.data?.message || '로그인에 실패했습니다.'
        showError(errorMessage)
      } else if (error.request) {
        // 네트워크 오류 (서버에 도달하지 못함)
        showError('서버에 연결할 수 없습니다. 네트워크 연결을 확인해주세요.')
      } else {
        // 기타 오류
        showError('로그인 중 오류가 발생했습니다. 다시 시도해주세요.')
      }
    } finally {
      isLoading.value = false
    }
  }
  
  const goBack = () => {
    router.go(-1)
  }
  
  const goToMain = () => {
    router.push('/main')
  }
  
  // 컴포넌트 마운트 시 저장된 이메일 불러오기
  onMounted(() => {
    const savedEmail = localStorage.getItem('adminEmail')
    if (savedEmail) {
      formData.value.email = savedEmail
      formData.value.rememberMe = true
    }
  })
  </script>
  
  <style scoped>
  .login-container {
    display: flex;
    height: 100vh;
    background-color: #1a1f2e;
    color: #ffffff;
  }
  
  /* 뒤로가기 버튼 */
  .back-button {
    position: absolute;
    top: 20px;
    left: 20px;
    background: none;
    border: none;
    color: #ffffff;
    font-size: 24px;
    cursor: pointer;
    padding: 8px;
    border-radius: 50%;
    transition: background-color 0.3s ease;
    z-index: 10;
  }
  
  .back-button:hover {
    background-color: rgba(255, 255, 255, 0.1);
  }
  
  /* 로그인 폼 영역 */
  .login-form {
    flex: 1;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    padding: 40px;
    max-width: 680px;
    margin: 0 auto;
  }
  
  /* 로고 섹션 */
  .logo-section {
    margin-bottom: 40px;
    display: flex;
    align-items: center;
    justify-content: center;
  
  }
  
  .logo-placeholder {
    width: 80px;
    height: 80px;
    border-radius: 50%;
    background-color: #2a2f3e;
    display: flex;
    align-items: center;
    justify-content: center;
    margin: 0;
  }
  
  .logo-icon {
    font-size: 40px;
  }
  
  /* 로그인 제목 */
  .login-title {
    font-size: 32px;
    font-weight: bold;
    margin-bottom: 40px;
    text-align: center !important;
  }
  
  /* 메시지 스타일 */
  .error-message {
    background-color: #dc3545;
    color: white;
    padding: 12px 16px;
    border-radius: 8px;
    margin-bottom: 20px;
    font-size: 14px;
    text-align: center;
    animation: slideIn 0.3s ease;
  }
  
  .success-message {
    background-color: #28a745;
    color: white;
    padding: 12px 16px;
    border-radius: 8px;
    margin-bottom: 20px;
    font-size: 14px;
    text-align: center;
    animation: slideIn 0.3s ease;
  }
  
  @keyframes slideIn {
    from {
      opacity: 0;
      transform: translateY(-10px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
  
  /* 폼 스타일 */
  .form {
    width: 100%;
    max-width: 544px;
  }
  
  .input-group {
    margin-bottom: 24px;
  }
  
  .input-label {
    display: block;
    margin-bottom: 8px;
    font-size: 14px;
    color: #b6bace;
    font-weight: 500;
  }
  
  .input-field {
    width: 94%;
    padding: 12px 16px;
    border: 1px solid #2a2f3e;
    border-radius: 8px;
    background-color: #222738;
    color: #ffffff;
    font-size: 16px;
    transition: border-color 0.3s ease;
  }
  
  .input-field:focus {
    outline: none;
    border-color: #3a57e8;
  }
  
  .input-field::placeholder {
    color: #8a92a6;
  }
  
  /* 비밀번호 컨테이너 */
  .password-container {
    position: relative;
    display: flex;
    align-items: center;
  }
  
  .password-input {
    padding-right: 80px;
  }
  
  .password-toggle {
    position: absolute;
    right: 8px;
    background: none;
    border: none;
    color: #3a57e8;
    font-size: 14px;
    cursor: pointer;
    padding: 8px 12px;
    border-radius: 4px;
    transition: background-color 0.3s ease;
  }
  
  .password-toggle:hover {
    background-color: rgba(58, 87, 232, 0.1);
  }
  
  /* 체크박스 그룹 */
  .checkbox-group {
    margin-bottom: 24px;
  }
  
  .checkbox-label {
    display: flex;
    align-items: center;
    cursor: pointer;
  }
  
  .checkbox {
    margin-right: 12px;
    width: 18px;
    height: 18px;
    accent-color: #3a57e8;
  }
  
  .checkbox-text {
    font-size: 14px;
    color: #b6bace;
  }
  
  /* 이용약관 텍스트 */
  .terms-text {
    font-size: 12px;
    color: #8a92a6;
    text-align: center;
    margin-bottom: 32px;
    line-height: 1.5;
  }
  
  .terms-link {
    color: #3a57e8;
    text-decoration: none;
    transition: color 0.3s ease;
  }
  
  .terms-link:hover {
    color: #4a67f8;
  }
  
  /* 로그인 버튼 */
  .login-button {
    width: 100%;
    padding: 16px;
    background-color: #1e40af;
    color: #ffffff;
    border: none;
    border-radius: 8px;
    font-size: 16px;
    font-weight: 600;
    cursor: pointer;
    transition: background-color 0.3s ease;
    margin-bottom: 24px;
  }
  
  .login-button:hover:not(:disabled) {
    background-color: #1d4ed8;
  }
  
  .login-button:disabled {
    background-color: #4b5563;
    cursor: not-allowed;
  }
  
  /* 로딩 스피너 */
  .loading-spinner {
    display: inline-block;
    width: 16px;
    height: 16px;
    border: 2px solid #ffffff;
    border-radius: 50%;
    border-top-color: transparent;
    animation: spin 1s linear infinite;
    margin-right: 8px;
  }
  
  @keyframes spin {
    to {
      transform: rotate(360deg);
    }
  }
  
  /* 추가 링크들 */
  .additional-links {
    display: flex;
    flex-direction: column;
    gap: 12px;
    text-align: center;
  }
  
  .link {
    color: #3a57e8;
    text-decoration: none;
    font-size: 14px;
    transition: color 0.3s ease;
  }
  
  .link:hover {
    color: #4a67f8;
  }
  
  /* 우측 검은 영역 */
  .right-section {
    width: 33.33%;
    background-color: #000000;
    position: relative;
  }
  
  .red-dot {
    position: absolute;
    bottom: 40px;
    right: 40px;
    width: 12px;
    height: 12px;
    background-color: #ff4757;
    border-radius: 50%;
  }
  
  /* 반응형 디자인 */
  @media (max-width: 768px) {
    .login-container {
      flex-direction: column;
    }
    
    .right-section {
      width: 100%;
      height: 100px;
    }
    
    .login-form {
      padding: 20px;
      max-width: 100%;
      align-items: center;
    }
    
    .form {
      max-width: 100%;
    }
    
    .back-button {
      top: 10px;
      left: 10px;
    }
  }
  
  @media (max-width: 480px) {
    .login-title {
      font-size: 28px;
      text-align: center !important;
    }
    
    .logo-placeholder {
      width: 60px;
      height: 60px;
    }
    
    .logo-icon {
      font-size: 30px;
    }
    
    .input-field {
      font-size: 14px;
      padding: 10px 14px;
    }
    
    .login-button {
      padding: 14px;
      font-size: 14px;
    }
  }
  </style>