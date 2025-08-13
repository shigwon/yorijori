// 세션 관리 유틸리티 함수들

// 세션 상태 확인
export const isLoggedIn = () => {
  const adminSession = localStorage.getItem('adminSession') || sessionStorage.getItem('adminSession')
  const adminEmail = localStorage.getItem('adminEmail') || sessionStorage.getItem('adminEmail')
  return adminSession === 'true' && adminEmail
}

// 현재 로그인된 사용자 이메일 가져오기
export const getCurrentUser = () => {
  return localStorage.getItem('adminEmail') || sessionStorage.getItem('adminEmail')
}

// 세션 정보 저장
export const setSession = (email, rememberMe = false) => {
  if (rememberMe) {
    localStorage.setItem('adminEmail', email)
    localStorage.setItem('adminSession', 'true')
  } else {
    sessionStorage.setItem('adminEmail', email)
    sessionStorage.setItem('adminSession', 'true')
  }
}

// 세션 정보 삭제
export const clearSession = () => {
  localStorage.removeItem('adminSession')
  localStorage.removeItem('adminEmail')
  sessionStorage.removeItem('adminSession')
  sessionStorage.removeItem('adminEmail')
}

// 세션 만료 시간 설정 (선택사항)
export const setSessionExpiry = (minutes = 30) => {
  const expiryTime = new Date().getTime() + (minutes * 60 * 1000)
  localStorage.setItem('adminSessionExpiry', expiryTime.toString())
}

// 세션 만료 확인
export const isSessionExpired = () => {
  const expiryTime = localStorage.getItem('adminSessionExpiry')
  if (!expiryTime) return false
  
  const currentTime = new Date().getTime()
  const isExpired = currentTime > parseInt(expiryTime)
  
  if (isExpired) {
    clearSession()
    return true
  }
  
  return false
}

// 세션 갱신
export const refreshSession = () => {
  if (isLoggedIn()) {
    setSessionExpiry(30) // 30분으로 갱신
    return true
  }
  return false
} 