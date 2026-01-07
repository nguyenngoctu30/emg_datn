// Common JS for basic interactions (preloader + mobile menu toggle)
(function () {
  // Mobile menu toggle (safe)
  document.addEventListener('DOMContentLoaded', () => {
    const toggle = document.getElementById('menu-toggle');
    const menu = document.getElementById('mobile-menu');
    if (toggle && menu) {
      toggle.addEventListener('click', () => {
        menu.classList.toggle('hidden');
      });
    }
  });

  // Preloader fade out on full load
  window.addEventListener('load', () => {
    const preloader = document.getElementById('preloader');
    if (!preloader) {
      document.body.classList.remove('preloading');
      return;
    }
    setTimeout(() => {
      preloader.classList.add('fade-out');
      setTimeout(() => {
        preloader.style.display = 'none';
        document.body.classList.remove('preloading');
      }, 600);
    }, 300);
  });
})();