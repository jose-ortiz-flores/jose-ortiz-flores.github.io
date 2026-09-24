document.getElementById('year').textContent = new Date().getFullYear();

const observer = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    if (entry.isIntersecting) {
      entry.target.classList.add('visible');
      observer.unobserve(entry.target);
    }
  });
}, { threshold: 0.12 });

document.querySelectorAll('.reveal').forEach((el) => observer.observe(el));

const carousel = document.getElementById('project-carousel');
const prev = document.getElementById('projects-prev');
const next = document.getElementById('projects-next');
const status = document.getElementById('project-position');

if (carousel && prev && next) {
  const cards = Array.from(carousel.querySelectorAll('.project-card'));
  const visibleCount = () => window.innerWidth <= 650 ? 1 : window.innerWidth <= 960 ? 2 : 3;
  const cardStep = () => {
    if (!cards.length) return 0;
    const styles = getComputedStyle(carousel);
    return cards[0].getBoundingClientRect().width + parseFloat(styles.columnGap || styles.gap || 0);
  };
  const currentIndex = () => Math.max(0, Math.round(carousel.scrollLeft / Math.max(cardStep(), 1)));
  const updateControls = () => {
    const count = visibleCount();
    const index = currentIndex();
    const lastStart = Math.max(0, cards.length - count);
    prev.disabled = index <= 0;
    next.disabled = index >= lastStart;
    const firstShown = Math.min(index + 1, cards.length);
    const lastShown = Math.min(index + count, cards.length);
    status.textContent = count === 1 ? `Project ${firstShown} of ${cards.length}` : `Projects ${firstShown}–${lastShown} of ${cards.length}`;
  };
  prev.addEventListener('click', () => carousel.scrollBy({left: -cardStep(), behavior:'smooth'}));
  next.addEventListener('click', () => carousel.scrollBy({left: cardStep(), behavior:'smooth'}));
  carousel.addEventListener('scroll', () => requestAnimationFrame(updateControls), {passive:true});
  window.addEventListener('resize', updateControls);
  updateControls();
}
